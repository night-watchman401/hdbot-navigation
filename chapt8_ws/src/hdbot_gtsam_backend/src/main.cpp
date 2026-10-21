// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#include "hdbot_gtsam_backend/slam_gnss_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = hdbot_gtsam_backend::makeSlamGnssNode(rclcpp::NodeOptions());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
