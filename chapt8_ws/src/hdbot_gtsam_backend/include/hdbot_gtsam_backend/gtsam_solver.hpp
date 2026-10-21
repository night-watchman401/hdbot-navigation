// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "hdbot_gtsam_backend/graph_backend.hpp"
#include <karto_sdk/Mapper.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <hdbot_gnss_msgs/msg/doppler_epoch.hpp>
#include <map>
#include <mutex>
#include <unordered_map>

namespace hdbot_gtsam_backend
{
class GtsamSolver : public karto::ScanSolver
{
public:
  void Configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node) override;
  void AddNode(karto::Vertex<karto::LocalizedRangeScan> * vertex) override;
  void AddConstraint(karto::Edge<karto::LocalizedRangeScan> * edge) override;
  void RemoveNode(kt_int32s id) override;
  void RemoveConstraint(kt_int32s first, kt_int32s second) override;
  void Compute() override;
  const IdPoseVector & GetCorrections() const override {return corrections_;}
  // Karto calls Clear after applying corrections. It MUST NOT erase the graph.
  void Clear() override;
  void Reset() override;
  std::unordered_map<int, Eigen::Vector3d> * getGraph() override {return &display_graph_;}
  void ModifyNode(const int & id, Eigen::Vector3d pose) override;
  void GetNodeOrientation(const int & id, double & yaw) override;
  bool pending() const;
  SolveReport lastReport() const;

private:
  void receiveEpoch(hdbot_gnss_msgs::msg::DopplerEpoch::ConstSharedPtr message);
  std::vector<MatchedEpoch> matchedEpochs();
  mutable std::mutex mutex_;
  GraphBackend backend_;
  IdPoseVector corrections_;
  std::unordered_map<int, Eigen::Vector3d> display_graph_;
  std::map<double, gtsam::Vector3> imu_;
  std::map<double, double> wheel_;
  std::map<double, hdbot_gnss_msgs::msg::DopplerEpoch> epochs_;
  std::map<double, MatchedEpoch> matched_cache_;
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<hdbot_gnss_msgs::msg::DopplerEpoch>::SharedPtr gnss_subscription_;
  SolveReport last_report_;
  bool dirty_{false}, enabled_{true}, georeference_ready_{false};
  double sync_tolerance_{0.15}, pose_tolerance_{0.15};
  int minimum_satellites_{4}, max_epochs_{2000};
  std::string base_frame_, imu_frame_, antenna_frame_;
  gtsam::Rot3 body_from_imu_;
  gtsam::Vector3 gyro_bias_{0, 0, 0};
};
}  // namespace hdbot_gtsam_backend
