// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#include "hdbot_gtsam_backend/gtsam_solver.hpp"
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

// Karto's Clear callback must leave nodes available for the next optimization.
TEST(Solver, ClearAndResetHaveDifferentSemantics)
{
  hdbot_gtsam_backend::GtsamSolver solver;
  karto::LocalizedRangeScan scan(karto::Name("laser"), std::vector<kt_double>{1.0});
  scan.SetUniqueId(1); scan.SetTime(1.0); scan.SetCorrectedPose({1.0, 2.0, 0.3});
  karto::Vertex<karto::LocalizedRangeScan> vertex(&scan);
  solver.AddNode(&vertex); solver.Compute();
  ASSERT_TRUE(solver.lastReport().success);
  ASSERT_EQ(solver.GetCorrections().size(), 1u);
  solver.Clear(); EXPECT_TRUE(solver.GetCorrections().empty());
  solver.Compute(); EXPECT_EQ(solver.GetCorrections().size(), 1u);
  solver.RemoveNode(1); EXPECT_TRUE(solver.getGraph()->empty());
  solver.Reset(); EXPECT_FALSE(solver.pending());
}

// Loading through the same registry as slam_toolbox catches plugin XML/export errors.
TEST(Solver, PluginRegistration)
{
  pluginlib::ClassLoader<karto::ScanSolver> loader("slam_toolbox", "karto::ScanSolver");
  EXPECT_TRUE(loader.isClassAvailable("hdbot_gtsam_backend::GtsamSolver"));
  auto instance = loader.createSharedInstance("hdbot_gtsam_backend::GtsamSolver");
  EXPECT_NE(dynamic_cast<hdbot_gtsam_backend::GtsamSolver *>(instance.get()), nullptr);
}

// Real ROS subscriptions must synchronize raw epochs, add factors and reject stale data.
TEST(Solver, RosDopplerInput)
{
  using namespace hdbot_gtsam_backend;
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.parameter_overrides({
    rclcpp::Parameter("gnss.georeference_ready", true),
    rclcpp::Parameter("gnss.ecef_from_map_translation_m", std::vector<double>{6378137, 0, 0}),
    rclcpp::Parameter("gnss.lever_arm_body_m", std::vector<double>{0.5, -0.2, 0.1})});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("solver_test", options);
  GtsamSolver solver; solver.Configure(node);
  auto publisher = std::make_shared<rclcpp::Node>("observations_test",
    rclcpp::NodeOptions().use_intra_process_comms(true));
  auto imu = publisher->create_publisher<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS());
  auto odom = publisher->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS());
  auto gnss = publisher->create_publisher<hdbot_gnss_msgs::msg::DopplerEpoch>(
    "/gnss/doppler", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface()); executor.add_node(publisher);
  std::vector<std::unique_ptr<karto::LocalizedRangeScan>> scans;
  std::vector<std::unique_ptr<karto::Vertex<karto::LocalizedRangeScan>>> vertices;
  std::vector<std::unique_ptr<karto::Edge<karto::LocalizedRangeScan>>> edges;
  for (int k = 0; k < 3; ++k) {
    scans.push_back(std::make_unique<karto::LocalizedRangeScan>(karto::Name("laser"), std::vector<kt_double>{1}));
    scans.back()->SetUniqueId(k); scans.back()->SetTime(k+1);
    scans.back()->SetCorrectedPose({0.8*k, 0, 0});
    vertices.push_back(std::make_unique<karto::Vertex<karto::LocalizedRangeScan>>(scans.back().get()));
    solver.AddNode(vertices.back().get());
    if (k) {
      edges.push_back(std::make_unique<karto::Edge<karto::LocalizedRangeScan>>(
        vertices[k-1].get(), vertices[k].get()));
      karto::Matrix3 covariance; covariance.SetToIdentity();
      edges.back()->SetLabel(new karto::LinkInfo({0,0,0}, {0.8,0,0}, covariance));
      solver.AddConstraint(edges.back().get());
    }
  }
  for (int k : {2, 0, 1}) {
    const builtin_interfaces::msg::Time stamp = rclcpp::Time(static_cast<int64_t>(k+1)*1000000000LL);
    sensor_msgs::msg::Imu imu_message;
    imu_message.header.stamp = stamp; imu_message.header.frame_id = "imu_link";
    imu_message.angular_velocity.z = 0.4;
    nav_msgs::msg::Odometry odom_message;
    odom_message.header.stamp = stamp; odom_message.child_frame_id = "base_footprint";
    odom_message.twist.twist.linear.x = 0.8;
    hdbot_gnss_msgs::msg::DopplerEpoch epoch;
    epoch.header.stamp = stamp; epoch.header.frame_id = "gnss_antenna";
    epoch.receiver_position_ecef.x = 6378137;
    int signal = 0;
    for (const auto & direction : std::vector<gtsam::Vector3>{
      {1,1,1}, {-1,1,1}, {1,-1,1}, {1,1,-1}, {-1,-1,1}, {-1,1,-1}}) {
      const gtsam::Point3 receiver(6378137,0,0), position = receiver + 2e7*direction.normalized();
      const gtsam::Point3 velocity(1000,-800,400);
      gtsam::DopplerFactorArm official(0,1,2,3,0,0.19,position,velocity,receiver,
        {0.5,-0.2,0.1},gtsam::Pose3(gtsam::Rot3(),receiver),{0,0,0.4},1,0);
      hdbot_gnss_msgs::msg::DopplerObservation observation;
      observation.signal_id = std::to_string(signal++); observation.valid = true;
      observation.wavelength_m = 0.19; observation.range_rate_sigma_mps = 0.02;
      observation.doppler_hz = -official.evaluateError(gtsam::Pose3(),
        gtsam::Vector3(0.8,0,0),0,0)[0]/0.19;
      observation.satellite_position_ecef.x = position.x();
      observation.satellite_position_ecef.y = position.y();
      observation.satellite_position_ecef.z = position.z();
      observation.satellite_velocity_ecef.x = velocity.x();
      observation.satellite_velocity_ecef.y = velocity.y();
      observation.satellite_velocity_ecef.z = velocity.z();
      epoch.observations.push_back(observation);
    }
    imu->publish(imu_message); odom->publish(odom_message); gnss->publish(epoch);
    for (int i = 0; i < 10; ++i) {
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  solver.Compute();
  ASSERT_TRUE(solver.lastReport().success) << solver.lastReport().message;
  EXPECT_EQ(solver.lastReport().doppler_factors, 12u);
  EXPECT_LT(solver.lastReport().doppler_rms, 1e-6);
  EXPECT_EQ(solver.lastReport().pose_edges, 2u);
  // Deleting a node must also delete its original graph edges and its GNSS attachment.
  solver.RemoveNode(2); solver.Compute();
  EXPECT_TRUE(solver.lastReport().success);
  EXPECT_EQ(solver.lastReport().doppler_factors, 6u);
  EXPECT_EQ(solver.lastReport().pose_edges, 1u);
}
