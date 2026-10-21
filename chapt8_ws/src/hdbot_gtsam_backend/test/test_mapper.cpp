// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#include "hdbot_gtsam_backend/slam_gnss_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <gtest/gtest.h>

// Exercise the actual mapper's lifecycle and plugin loading without DDS services.
TEST(Mapper, ConfigureActivateDeactivateAndReconfigure)
{
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("solver_plugin", "hdbot_gtsam_backend::GtsamSolver"),
    rclcpp::Parameter("use_sim_time", false),
    rclcpp::Parameter("gnss.optimization_period_s", 0.05)});
  auto node = hdbot_gtsam_backend::makeSlamGnssNode(options);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin_some();
  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}
