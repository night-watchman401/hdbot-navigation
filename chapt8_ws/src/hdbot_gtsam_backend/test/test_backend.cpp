// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#include "hdbot_gtsam_backend/graph_backend.hpp"
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/GnssCommon.h>
#include <gtest/gtest.h>

using namespace hdbot_gtsam_backend;
using gtsam::Symbol;

namespace
{
PlanarDopplerFactor factor(const gtsam::Vector3 & rate, const gtsam::Vector3 & arm)
{
  return {0, 1, 2, 3, -1500, 0.190293672798365,
    {-5824269.46, -22935011.27, -12195522.22}, {-1200, 2400, 800},
    {-3961908.12, 3348995.59, 3698211.13}, arm,
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.1, 0.4, -0.7), {6378137, 0, 0}),
    rate, 0.2, 1.2e-9, gtsam::noiseModel::Unit::Create(1)};
}
std::vector<MatchedEpoch> syntheticEpochs(double yaw)
{
  const gtsam::Point3 receiver(6378137, 0, 0), arm(0.5, -0.2, 0.1), rate(0, 0, 0.4);
  const gtsam::Pose3 ecef_from_map(gtsam::Rot3(), receiver);
  const gtsam::Pose3 pose(gtsam::Rot3::Rz(yaw), {0, 0, 0});
  const gtsam::Vector3 velocity(0.8 * std::cos(yaw), 0.8 * std::sin(yaw), 0);
  std::vector<MatchedEpoch> epochs;
  for (int k = 0; k < 3; ++k) {
    MatchedEpoch epoch;
    epoch.time = 1.0 + k; epoch.pose_id = k; epoch.wheel_speed = 0.8;
    epoch.receiver_position = receiver; epoch.angular_velocity = rate;
    int id = 0;
    for (const auto & direction : std::vector<gtsam::Point3>{
      {1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {1, 1, -1}, {-1, -1, 1}, {-1, 1, -1}}) {
      const gtsam::Point3 satellite = receiver + 2e7 * direction.normalized();
      const gtsam::Point3 sat_velocity(1000, -800, 400);
      gtsam::DopplerFactorArm official(0, 1, 2, 3, 0.0, 0.19,
        satellite, sat_velocity, receiver, arm, ecef_from_map, rate, 1.0, 1e-9);
      const double prediction = official.evaluateError(pose, velocity, 0.0, 2e-9)[0];
      epoch.satellites.push_back({std::to_string(id++), -prediction / 0.19, 0.19,
        1e-9, 0.02, satellite, sat_velocity});
    }
    epochs.push_back(epoch);
  }
  return epochs;
}
GraphBackend syntheticGraph()
{
  GraphBackend backend;
  backend.options.ecef_from_map = gtsam::Pose3(gtsam::Rot3(), {6378137, 0, 0});
  backend.options.lever_arm = {0.5, -0.2, 0.1};
  backend.options.body_velocity_sigmas = {0.02, 0.02, 0.02};
  for (int k = 0; k < 3; ++k) {
    backend.nodes[k] = {1.0+k, gtsam::Pose2(0.8*k, 0, 0)};
    if (k) {backend.edges.push_back({k-1, k, {0.8, 0, 0},
      (gtsam::Vector3(0.1, 0.1, 0.1).array().square().matrix()).asDiagonal()});}
  }
  backend.edges.push_back({0, 2, {1.6, 0, 0}, gtsam::Matrix3::Identity() * 0.02});
  return backend;
}
}  // namespace

// The adapter must preserve all four official Jacobians after lifting/scaling.
TEST(Doppler, AnalyticJacobians)
{
  const auto f = factor({0.02, -0.05, 0.3}, {0.5, -0.3, 1.0});
  const gtsam::Pose2 pose(1, -2, 0.7);
  const gtsam::Vector3 v(0.3, -0.1, 0.05);
  gtsam::Matrix Hx, Hv, Hp, Hc;
  f.evaluateError(pose, v, 20.0, 20.3, &Hx, &Hv, &Hp, &Hc);
  const auto nx = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose2>(
    [&](const auto & x) {return f.evaluateError(x, v, 20.0, 20.3);}, pose);
  const auto nv = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
    [&](const auto & x) {return f.evaluateError(pose, x, 20.0, 20.3);}, v);
  const auto np = gtsam::numericalDerivative11<gtsam::Vector, double>(
    [&](double x) {return f.evaluateError(pose, v, x, 20.3);}, 20.0);
  const auto nc = gtsam::numericalDerivative11<gtsam::Vector, double>(
    [&](double x) {return f.evaluateError(pose, v, 20.0, x);}, 20.3);
  EXPECT_TRUE(Hx.isApprox(nx, 1e-5)); EXPECT_TRUE(Hv.isApprox(nv, 1e-5));
  EXPECT_TRUE(Hp.isApprox(np, 1e-6)); EXPECT_TRUE(Hc.isApprox(nc, 1e-6));
}

// No direct yaw information exists in each of these planar degeneracies.
TEST(Doppler, ZeroRateZeroArmAndVerticalArm)
{
  for (const auto & f : {factor({0, 0, 0}, {1, 0, 0}),
      factor({0, 0, 0.5}, {0, 0, 0}), factor({0, 0, 0.5}, {0, 0, 1})}) {
    gtsam::Matrix H;
    f.evaluateError({0, 0, 0.8}, {1, 2, 0}, 0, 0, &H);
    EXPECT_LT(H.norm(), 1e-12);
  }
}

// A free map velocity exactly absorbs attitude changes at one epoch.
TEST(Doppler, FreeVelocityAbsorbsYaw)
{
  const gtsam::Vector3 rate(0, 0, 0.3), arm(0.5, -0.3, 1), v(1, 2, 0);
  const auto f = factor(rate, arm);
  const gtsam::Pose2 a(0, 0, 0.2), b(0, 0, 0.9);
  const gtsam::Vector3 adjusted = v + gtsam::Rot3::Rz(a.theta()).rotate(rate.cross(arm)) -
    gtsam::Rot3::Rz(b.theta()).rotate(rate.cross(arm));
  EXPECT_NEAR(f.evaluateError(a, v, 0, 0)[0], f.evaluateError(b, adjusted, 0, 0)[0], 1e-9);
}

// Body-frame coupling uses Pose2's tangent convention rather than world dx/dy.
TEST(Velocity, Jacobians)
{
  BodyVelocityFactor f(0, 1, 0.4, gtsam::noiseModel::Unit::Create(3));
  const gtsam::Pose2 p(2, -3, 0.7); const gtsam::Vector3 v(0.5, -0.2, 0.1);
  gtsam::Matrix A, B; f.evaluateError(p, v, &A, &B);
  EXPECT_TRUE(A.isApprox(gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose2>(
    [&](const auto & x) {return f.evaluateError(x, v);}, p), 1e-7));
  EXPECT_TRUE(B.isApprox(gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
    [&](const auto & x) {return f.evaluateError(p, x);}, v), 1e-7));
}

// Karto edge Jacobians must agree away from the angle-wrap discontinuity.
TEST(PoseGraph, KartoJacobians)
{
  KartoBetweenFactor f(0, 1, {1, 0.2, 0.1}, gtsam::noiseModel::Unit::Create(3));
  const gtsam::Pose2 a(2, 3, 0.7), b(3, 5, 1.1);
  gtsam::Matrix A, B; f.evaluateError(a, b, &A, &B);
  EXPECT_TRUE(A.isApprox(gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose2>(
    [&](const auto & x) {return f.evaluateError(x, b);}, a), 1e-7));
  EXPECT_TRUE(B.isApprox(gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose2>(
    [&](const auto & x) {return f.evaluateError(a, x);}, b), 1e-7));
}

// GNSS and wheel coupling can correct a common heading error while retaining a loop edge.
TEST(Graph, HeadingImprovesAndLoopSurvives)
{
  auto backend = syntheticGraph();
  const auto report = backend.solve(syntheticEpochs(0.35));
  ASSERT_TRUE(report.success) << report.message;
  EXPECT_EQ(report.pose_edges, 3u); EXPECT_EQ(report.doppler_factors, 12u);
  EXPECT_LT(std::abs(backend.nodes.at(2).pose.theta() - 0.35), 0.03);
  EXPECT_LT(report.final_error, report.initial_error); EXPECT_LT(report.doppler_rms, 0.01);
  RecordProperty("initial_heading_error_rad", 0.35);
  RecordProperty("final_heading_error_rad", std::abs(backend.nodes.at(2).pose.theta() - 0.35));
  RecordProperty("doppler_rms_mps", report.doppler_rms);
}

// Disabling GNSS leaves the existing graph including its loop constraint intact.
TEST(Graph, NoGnssRetainsOriginalGraph)
{
  auto backend = syntheticGraph(); const auto report = backend.solve({});
  ASSERT_TRUE(report.success) << report.message;
  EXPECT_EQ(report.pose_edges, 3u); EXPECT_EQ(report.doppler_factors, 0u);
  EXPECT_NEAR(backend.nodes.at(2).pose.x(), 1.6, 1e-8);
}

// Out-of-order input is sorted and a gap creates a distinct anchored clock segment.
TEST(Graph, EpochGapsAndDuplicateTimes)
{
  auto backend = syntheticGraph(); auto epochs = syntheticEpochs(0.2);
  auto later = epochs.back(); later.time = 20; epochs.push_back(later);
  later.time = 21; epochs.push_back(later); epochs.push_back(later);
  std::reverse(epochs.begin(), epochs.end());
  const auto report = backend.solve(epochs);
  ASSERT_TRUE(report.success) << report.message;
  EXPECT_EQ(report.doppler_factors, 18u);
}

// Repeated periodic solves must use a stable prior instead of re-centering it.
TEST(Graph, RepeatedSolveKeepsGauge)
{
  auto backend = syntheticGraph(); const auto epochs = syntheticEpochs(0.35);
  ASSERT_TRUE(backend.solve(epochs).success);
  const double yaw = backend.nodes.at(2).pose.theta();
  ASSERT_TRUE(backend.solve(epochs).success);
  EXPECT_NEAR(backend.nodes.at(2).pose.theta(), yaw, 1e-5);
}

// Deletion can split a graph; each remaining component needs an explicit gauge.
TEST(Graph, DisconnectedComponentsAndReset)
{
  auto backend = syntheticGraph(); backend.edges.clear();
  ASSERT_TRUE(backend.solve({}).success);
  EXPECT_EQ(backend.nodes.size(), 3u);
  backend.reset(); EXPECT_TRUE(backend.nodes.empty()); EXPECT_TRUE(backend.values().empty());
}
