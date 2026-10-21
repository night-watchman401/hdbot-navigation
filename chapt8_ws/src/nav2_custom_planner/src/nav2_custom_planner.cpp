// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#include "nav2_util/node_utils.hpp"
// [修改05/08] 分别用于限制插值段数、检查整数范围，以及锁定代价地图。
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

// [修改01] Jazzy 将规划器异常放在 planner_exceptions.hpp 中。
#include "nav2_core/planner_exceptions.hpp"
// [修改02] 显式包含 LETHAL_OBSTACLE 的定义，不依赖其他头文件间接包含。
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_custom_planner/nav2_custom_planner.hpp"

namespace nav2_custom_planner
{

    void CustomPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        // [修改03] weak_ptr::lock() 可能返回空指针，先检查再使用节点。
        costmap_ = nullptr;
        node_ = parent.lock();
        if (!node_)
        {
            throw nav2_core::PlannerException("配置失败：父节点已经失效");
        }
        if (!costmap_ros || !costmap_ros->getCostmap())
        {
            throw nav2_core::PlannerException("配置失败：代价地图为空");
        }
        tf_ = tf;
        name_ = name;
        global_frame_ = costmap_ros->getGlobalFrameID();
        // 参数初始化
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution",
                             interpolation_resolution_);
        // [修改04] 分辨率必须是有限正数，避免后续除零、负循环次数或 NaN。
        if (!std::isfinite(interpolation_resolution_) || interpolation_resolution_ <= 0.0)
        {
            throw nav2_core::PlannerException(
                "interpolation_resolution 必须是有限正数");
        }
        // [修改03/04] 配置验证成功后才保存地图指针，失败时 createPlan() 会拒绝运行。
        costmap_ = costmap_ros->getCostmap();
        // [修改04] 输出实际加载的参数，方便确认 YAML 是否生效。
        RCLCPP_INFO(node_->get_logger(), "插件 %s 配置完成：frame=%s，resolution=%.3f",
                    name_.c_str(), global_frame_.c_str(), interpolation_resolution_);
    }

    void CustomPlanner::cleanup()
    {
        // [修改03] cleanup() 释放本插件持有的引用，并清空借用的地图指针。
        if (node_)
        {
            RCLCPP_INFO(node_->get_logger(), "正在清理类型为 CustomPlanner 的插件 %s",
                        name_.c_str());
        }
        costmap_ = nullptr;
        tf_.reset();
        node_.reset();
    }

    void CustomPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在激活类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    void CustomPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在停用类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    nav_msgs::msg::Path
    CustomPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                              const geometry_msgs::msg::PoseStamped &goal,
                              // [修改01] 必须与 Jazzy 基类和 .hpp 中的声明一致。
                              std::function<bool()> cancel_checker)
    {
        // [修改03] 防止未完成 configure() 或 cleanup() 后继续使用无效成员。
        if (!node_ || !costmap_)
        {
            throw nav2_core::PlannerException("插件尚未配置，或已完成清理");
        }
        // [修改01] 进入规划时先响应取消；循环中也会检查，避免长任务无法中止。
        if (cancel_checker && cancel_checker())
        {
            throw nav2_core::PlannerCancelled("规划请求已取消");
        }
        // 1.声明并初始化 global_path
        nav_msgs::msg::Path global_path;
        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        // 2.检查目标和起始状态是否在全局坐标系中
        if (start.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(node_->get_logger(), "规划器仅接受来自 %s 坐标系的起始位置",
                         global_frame_.c_str());
            // [修改04] 报告具体的坐标系错误，避免仅返回空路径掩盖失败原因。
            throw nav2_core::PlannerTFError("起点坐标系与全局坐标系不一致");
        }

        if (goal.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(node_->get_logger(), "规划器仅接受来自 %s 坐标系的目标位置",
                        global_frame_.c_str());
            // [修改04] 目标坐标系不匹配时同样明确报告 TF 类错误。
            throw nav2_core::PlannerTFError("目标坐标系与全局坐标系不一致");
        }

        // [修改04] 拒绝无效平面坐标，避免 NaN/Inf 进入插值和栅格坐标转换。
        if (!std::isfinite(start.pose.position.x) || !std::isfinite(start.pose.position.y) ||
            !std::isfinite(goal.pose.position.x) || !std::isfinite(goal.pose.position.y))
        {
            throw nav2_core::PlannerException("起点或目标的平面坐标不是有限数值");
        }

        const double dx = goal.pose.position.x - start.pose.position.x;
        const double dy = goal.pose.position.y - start.pose.position.y;
        const double distance = std::hypot(dx, dy);
        if (!std::isfinite(distance))
        {
            // [修改04] 即使输入各自有限，极端数值相减也可能溢出。
            throw nav2_core::PlannerException("起点到目标的距离超出有效数值范围");
        }

        // [修改05] 起终点重合时不做插值，只在后面添加并检查目标点，避免除零。
        if (distance > 0.0)
        {
            // [修改05] 向上取整使实际点间距不超过参数值；短距离至少分成一段。
            const double required_steps = std::ceil(distance / interpolation_resolution_);
            if (!std::isfinite(required_steps) ||
                required_steps > static_cast<double>(std::numeric_limits<int>::max()))
            {
                // [修改05] 先检查再转 int，避免极小分辨率导致浮点转整数溢出。
                throw nav2_core::PlannerException("插值段数过大，请增大分辨率或缩短距离");
            }
            const int total_number_of_loop = std::max(1, static_cast<int>(required_steps));
            const double x_increment = dx / total_number_of_loop;
            const double y_increment = dy / total_number_of_loop;
            // [修改06] 中间点朝向直线前进方向；平面 yaw 转换为单位四元数。
            const double yaw = std::atan2(dy, dx);

            for (int i = 0; i < total_number_of_loop; ++i)
            {
                // [修改01] 生成路径期间响应 Nav2 的取消请求。
                if (cancel_checker && cancel_checker())
                {
                    throw nav2_core::PlannerCancelled("规划请求已取消");
                }
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = start.pose.position.x + x_increment * i;
                pose.pose.position.y = start.pose.position.y + y_increment * i;
                pose.pose.position.z = 0.0;
                // [修改06] 原代码默认四元数全为 0，无法表示有效姿态。
                pose.pose.orientation.z = std::sin(yaw / 2.0);
                pose.pose.orientation.w = std::cos(yaw / 2.0);
                // [修改06] 同一条路径统一使用生成时的时间戳和坐标系。
                pose.header = global_path.header;
                global_path.poses.push_back(pose);
            }
        }

        // [修改07] 先加入目标点，再检查整条路径，修复原来目标点漏检的问题。
        // 保留调用方指定的目标姿态，使机器人最后到达时仍采用请求中的朝向。
        geometry_msgs::msg::PoseStamped goal_pose = goal;
        goal_pose.header = global_path.header;
        global_path.poses.push_back(goal_pose);

        // [修改08] 读取地图时持有地图自身的互斥锁，防止与地图更新线程并发读写。
        // unique_lock 离开作用域（包括抛异常）时会自动解锁。
        {
            std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
            // [修改08] 使用 const 引用遍历，避免每检查一个点就复制一次消息。
            for (const auto &pose : global_path.poses)
            {
                // [修改01] 检查代价地图期间也响应取消。
                if (cancel_checker && cancel_checker())
                {
                    throw nav2_core::PlannerCancelled("规划请求已取消");
                }
                unsigned int mx = 0, my = 0;
                if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
                {
                    // [修改08] 转换失败表示路径点越界，不能跳过后假装规划成功。
                    throw nav2_core::NoValidPathCouldBeFound(
                        "路径点超出代价地图范围: " + std::to_string(pose.pose.position.x) + "," +
                        std::to_string(pose.pose.position.y));
                }
                const unsigned char cost = costmap_->getCost(mx, my);
                // 本示例检查采样点所在栅格的致命障碍物，没有实现完整机器人足迹检测。
                if (cost == nav2_costmap_2d::LETHAL_OBSTACLE)
                {
                    RCLCPP_WARN(node_->get_logger(), "在(%f,%f)检测到致命障碍物，规划失败。",
                                pose.pose.position.x, pose.pose.position.y);
                    // [修改08] 使用“没有有效路径”异常，让 Planner Server 明确报告结果。
                    throw nav2_core::NoValidPathCouldBeFound("直线路径经过致命障碍物");
                }
            }
        }
        // [修改08] 便于在开启 debug 日志时确认最终输出的路径点数量。
        RCLCPP_DEBUG(node_->get_logger(), "插件 %s 生成 %zu 个路径点",
                     name_.c_str(), global_path.poses.size());
        return global_path;
    }

} // namespace nav2_custom_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner,
                       nav2_core::GlobalPlanner)
