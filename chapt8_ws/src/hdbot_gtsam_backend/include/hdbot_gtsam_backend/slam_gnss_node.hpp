// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <memory>

namespace hdbot_gtsam_backend
{
/** Construct the mapper as a lifecycle node for execution or in-process tests. */
std::shared_ptr<rclcpp_lifecycle::LifecycleNode> makeSlamGnssNode(
  const rclcpp::NodeOptions & options);
}
