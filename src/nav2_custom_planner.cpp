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
#include <queue>
#include <vector>
#include <utility>

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
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".algorithm", rclcpp::ParameterValue("STRAIGHT"));
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
        node_->get_parameter(name_ + ".algorithm", algorithm_);
        node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
        if (algorithm_ != "STRAIGHT" && algorithm_ != "A_STAR" &&
            algorithm_ != "THETA_STAR" && algorithm_ != "DIJKSTRA")
        {
            throw nav2_core::PlannerException(
                "algorithm 必须是 STRAIGHT、A_STAR、THETA_STAR 或 DIJKSTRA");
        }
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

        // 搜索与路径生成期间锁定地图，保证栅格尺寸、原点与代价一致。
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
        auto check_cancel = [&]() {
            if (cancel_checker && cancel_checker()) {
                throw nav2_core::PlannerCancelled("规划请求已取消");
            }
        };
        const int width = static_cast<int>(costmap_->getSizeInCellsX());
        const int height = static_cast<int>(costmap_->getSizeInCellsY());
        auto free_cell = [&](int x, int y) {
            if (x < 0 || y < 0 || x >= width || y >= height) {return false;}
            const auto cost = costmap_->getCost(x, y);
            return cost == nav2_costmap_2d::NO_INFORMATION ? allow_unknown_ :
                cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        };
        unsigned int sx, sy, gx, gy;
        if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
            !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
            throw nav2_core::NoValidPathCouldBeFound("起点或目标超出地图范围");
        }
        if (!free_cell(sx, sy) || !free_cell(gx, gy)) {
            throw nav2_core::NoValidPathCouldBeFound("起点或目标位于不可通行栅格");
        }
        // Supercover 遍历线段经过的全部栅格；经过角点时检查两侧，禁止切角。
        auto visible = [&](double ax, double ay, double bx, double by) {
            check_cancel();
            const double resolution = costmap_->getResolution();
            ax = (ax - costmap_->getOriginX()) / resolution;
            ay = (ay - costmap_->getOriginY()) / resolution;
            bx = (bx - costmap_->getOriginX()) / resolution;
            by = (by - costmap_->getOriginY()) / resolution;
            int x = static_cast<int>(std::floor(ax)), y = static_cast<int>(std::floor(ay));
            const int ex = static_cast<int>(std::floor(bx)), ey = static_cast<int>(std::floor(by));
            const double dx = bx - ax, dy = by - ay;
            const int stepx = dx > 0 ? 1 : -1, stepy = dy > 0 ? 1 : -1;
            const double inf = std::numeric_limits<double>::infinity();
            const double tx = dx == 0 ? inf : std::abs(1.0 / dx);
            const double ty = dy == 0 ? inf : std::abs(1.0 / dy);
            double mx = dx == 0 ? inf : (dx > 0 ? x + 1 - ax : ax - x) * tx;
            double my = dy == 0 ? inf : (dy > 0 ? y + 1 - ay : ay - y) * ty;
            while (true) {
                check_cancel();
                if (!free_cell(x, y)) {return false;}
                if (x == ex && y == ey) {return true;}
                if (mx < my) {x += stepx; mx += tx;}
                else if (my < mx) {y += stepy; my += ty;}
                else {
                    if (!free_cell(x + stepx, y) || !free_cell(x, y + stepy)) {return false;}
                    x += stepx; y += stepy; mx += tx; my += ty;
                }
            }
        };
        using Point = std::pair<double, double>;
        std::vector<Point> points{{start.pose.position.x, start.pose.position.y}};
        if (algorithm_ != "STRAIGHT") {
            const size_t count = static_cast<size_t>(width) * height;
            const size_t source = static_cast<size_t>(sy) * width + sx;
            const size_t target = static_cast<size_t>(gy) * width + gx;
            std::vector<double> distance(count, std::numeric_limits<double>::infinity());
            std::vector<size_t> parent(count, count);
            auto center = [&](size_t id) {
                double x, y;
                costmap_->mapToWorld(id % width, id / width, x, y);
                return Point{x, y};
            };
            auto metric = [&](size_t a, size_t b) {
                const auto pa = center(a), pb = center(b);
                return std::hypot(pa.first - pb.first, pa.second - pb.second);
            };
            struct Entry {double priority; double cost; size_t id;};
            auto compare = [](const Entry &a, const Entry &b) {return a.priority > b.priority;};
            std::priority_queue<Entry, std::vector<Entry>, decltype(compare)> open(compare);
            distance[source] = 0.0;
            parent[source] = source;
            open.push({0.0, 0.0, source});
            while (!open.empty()) {
                check_cancel();
                const auto current = open.top(); open.pop();
                if (current.cost != distance[current.id]) {continue;}
                if (current.id == target) {break;}
                const int x = current.id % width, y = current.id / width;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if ((dx == 0 && dy == 0) || !free_cell(x + dx, y + dy)) {continue;}
                        if (dx && dy && (!free_cell(x + dx, y) || !free_cell(x, y + dy))) {continue;}
                        const size_t next = static_cast<size_t>(y + dy) * width + x + dx;
                        size_t via = current.id;
                        double candidate = distance[via] + metric(via, next);
                        if (algorithm_ == "THETA_STAR") {
                            const size_t ancestor = parent[current.id];
                            const auto a = center(ancestor), b = center(next);
                            if (visible(a.first, a.second, b.first, b.second)) {
                                const double shortcut = distance[ancestor] + metric(ancestor, next);
                                if (shortcut < candidate) {via = ancestor; candidate = shortcut;}
                            }
                        }
                        if (candidate < distance[next]) {
                            parent[next] = via; distance[next] = candidate;
                            const double heuristic = algorithm_ == "DIJKSTRA" ? 0.0 : metric(next, target);
                            open.push({candidate + heuristic, candidate, next});
                        }
                    }
                }
            }
            if (parent[target] == count) {
                throw nav2_core::NoValidPathCouldBeFound("搜索失败：起点与目标之间没有可通行路径");
            }
            std::vector<Point> reverse;
            for (size_t id = target;; id = parent[id]) {
                check_cancel();
                reverse.push_back(center(id));
                if (id == source) {break;}
            }
            points.insert(points.end(), reverse.rbegin(), reverse.rend());
        }
        points.emplace_back(goal.pose.position.x, goal.pose.position.y);
        for (size_t i = 1; i < points.size(); ++i) {
            check_cancel();
            const auto a = points[i - 1], b = points[i];
            if (!visible(a.first, a.second, b.first, b.second)) {
                throw nav2_core::NoValidPathCouldBeFound("路径线段经过不可通行栅格");
            }
            const double dx = b.first - a.first, dy = b.second - a.second;
            const double length = std::hypot(dx, dy);
            if (length == 0.0) {continue;}
            const double steps = std::ceil(length / interpolation_resolution_);
            if (!std::isfinite(steps) || steps > std::numeric_limits<int>::max()) {
                throw nav2_core::PlannerException("插值段数过大，请增大插值分辨率");
            }
            const int segments = std::max(1, static_cast<int>(steps));
            const double yaw = std::atan2(dy, dx);
            for (int j = 0; j < segments; ++j) {
                check_cancel();
                geometry_msgs::msg::PoseStamped pose;
                pose.header = global_path.header;
                pose.pose.position.x = a.first + dx * (static_cast<double>(j) / segments);
                pose.pose.position.y = a.second + dy * (static_cast<double>(j) / segments);
                pose.pose.orientation.z = std::sin(yaw / 2.0);
                pose.pose.orientation.w = std::cos(yaw / 2.0);
                global_path.poses.push_back(pose);
            }
        }
        auto goal_pose = goal;
        goal_pose.header = global_path.header;
        global_path.poses.push_back(goal_pose);
        // [修改08] 便于在开启 debug 日志时确认最终输出的路径点数量。
        RCLCPP_DEBUG(node_->get_logger(), "插件 %s 生成 %zu 个路径点",
                     name_.c_str(), global_path.poses.size());
        return global_path;
    }

} // namespace nav2_custom_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner,
                       nav2_core::GlobalPlanner)
