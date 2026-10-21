// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "hdbot_gtsam_backend/factors.hpp"
#include <gtsam/nonlinear/Values.h>
#include <map>
#include <vector>
#include <string>

namespace hdbot_gtsam_backend
{
struct SatelliteObservation
{
  std::string signal;
  double doppler{}, wavelength{}, clock_drift{}, sigma{};
  gtsam::Point3 position, velocity;
};
struct MatchedEpoch
{
  double time{};
  int pose_id{};
  gtsam::Point3 receiver_position, angular_velocity;
  double wheel_speed{};
  std::vector<SatelliteObservation> satellites;
};
struct PoseNode
{
  double time{};
  gtsam::Pose2 pose;
};
struct PoseEdge
{
  int first{}, second{};
  gtsam::Pose2 measurement;
  gtsam::Matrix3 covariance;
};
struct BackendOptions
{
  gtsam::Pose3 ecef_from_map;
  gtsam::Point3 lever_arm{0, 0, 0};
  gtsam::Vector3 body_velocity_sigmas{0.15, 0.15, 0.1};
  gtsam::Vector3 anchor_sigmas{0.05, 0.05, 3.14};
  double huber{1.345};
  double max_epoch_gap{3.0};
};
struct SolveReport
{
  bool success{};
  size_t pose_edges{}, doppler_factors{}, epochs{};
  double initial_error{}, final_error{}, doppler_rms{};
  std::string message;
};

/** Batch graph ownership is separate from the ROS subscriptions and Karto API.
 * All original pose edges survive every rebuild. Only accepted synchronized
 * GNSS epochs introduce velocity and clock states.
 */
class GraphBackend
{
public:
  std::map<int, PoseNode> nodes;
  std::vector<PoseEdge> edges;
  BackendOptions options;
  SolveReport solve(const std::vector<MatchedEpoch> & epochs);
  const gtsam::Values & values() const {return values_;}
  void reset();
  void modifyNode(int id, const gtsam::Pose2 & pose);
private:
  gtsam::Values values_;
  std::map<int, gtsam::Pose2> reference_poses_;
};
}  // namespace hdbot_gtsam_backend
