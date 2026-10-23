// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
// Test access without requiring lifecycle activation and a live TF tree.
#define private public
#include "nav2_custom_planner/nav2_custom_planner.hpp"
#undef private
#include <stdexcept>
#include <iostream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_smoke");
  int checks = 0;
  auto require = [&](bool ok) {
      if (!ok) {throw std::runtime_error("planner smoke check failed");}
      ++checks;
    };
  for (const std::string algorithm : {"A_STAR", "THETA_STAR", "DIJKSTRA"}) {
    nav2_costmap_2d::Costmap2D map(12, 12, 1.0, 0.0, 0.0, 0);
    nav2_custom_planner::CustomPlanner planner;
    planner.node_ = node;
    planner.costmap_ = &map;
    planner.global_frame_ = "map";
    planner.algorithm_ = algorithm;
    geometry_msgs::msg::PoseStamped start, goal;
    start.header.frame_id = goal.header.frame_id = "map";
    start.pose.position.x = start.pose.position.y = 1.5;
    goal.pose.position.x = 9.5;
    goal.pose.position.y = 1.5;
    goal.pose.orientation.w = 1.0;
    for (unsigned y = 0; y < 8; ++y) {map.setCost(5, y, 254);}
    const auto path = planner.createPlan(start, goal, [] {return false;});
    require(!path.poses.empty());
    require(path.poses.front().pose.position.x == 1.5);
    require(path.poses.back().pose.position.x == 9.5);
    bool detour = false;
    for (const auto &pose : path.poses) {
      unsigned x, y;
      require(map.worldToMap(pose.pose.position.x, pose.pose.position.y, x, y));
      require(map.getCost(x, y) < 253);
      detour = detour || pose.pose.position.y >= 8.0;
    }
    require(detour);
    for (unsigned y = 8; y < 12; ++y) {map.setCost(5, y, 254);}
    bool failed = false;
    try {planner.createPlan(start, goal, {});}
    catch (const nav2_core::NoValidPathCouldBeFound &) {failed = true;}
    require(failed);
    bool cancelled = false;
    try {planner.createPlan(start, goal, [] {return true;});}
    catch (const nav2_core::PlannerCancelled &) {cancelled = true;}
    require(cancelled);
    map.resetMap(0, 0, 12, 12);
    map.setCost(2, 1, 254);
    map.setCost(1, 2, 254);
    map.setCost(0, 1, 254);
    map.setCost(1, 0, 254);
    failed = false;
    try {planner.createPlan(start, goal, {});}
    catch (const nav2_core::NoValidPathCouldBeFound &) {failed = true;}
    require(failed);
  }
  std::cout << "Passed " << checks << " checks\n";
  rclcpp::shutdown();
}
