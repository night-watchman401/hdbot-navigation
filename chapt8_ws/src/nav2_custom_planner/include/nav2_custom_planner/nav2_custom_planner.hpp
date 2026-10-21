// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
#define NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
// [修改01] Jazzy 的 createPlan() 使用 std::function<bool()> 取消检查回调。
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
// [修改02] 直接包含本类实际使用的类型，移除未使用的 point 和 robot_utils 头文件。
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_custom_planner {
// 自定义导航规划器类
class CustomPlanner : public nav2_core::GlobalPlanner {
public:
  CustomPlanner() = default;
  ~CustomPlanner() = default;
  // 插件配置方法
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  // 插件清理方法
  void cleanup() override;
  // 插件激活方法
  void activate() override;
  // 插件停用方法
  void deactivate() override;
  // [修改01] Jazzy 的基类接口有第三个参数；声明和 .cpp 定义必须一致。
  // cancel_checker() 返回 true 时，表示这次规划请求已经被取消。
  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal,
             std::function<bool()> cancel_checker) override;

private:
  // 坐标变换缓存指针，可用于查询坐标关系
  std::shared_ptr<tf2_ros::Buffer> tf_;
  // [修改03] 显式写出与 parent.lock() 返回值一致的节点类型，便于理解所有权。
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  // [修改03] 地图由 Costmap2DROS 管理，本类只借用指针；初始为空，不负责 delete。
  nav2_costmap_2d::Costmap2D *costmap_{nullptr};
  // 全局代价地图的坐标系
  std::string global_frame_, name_;
  // [修改03] 给出确定的初值；configure() 随后从参数读取并验证它。
  double interpolation_resolution_{0.1};
};

} // namespace nav2_custom_planner

#endif // NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
