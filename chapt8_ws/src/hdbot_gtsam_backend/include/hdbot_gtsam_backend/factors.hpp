// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/navigation/DopplerFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <cmath>

namespace hdbot_gtsam_backend
{
constexpr double kLightSpeed = 299792458.0;

/** Restricts the official Pose3 factor to SE(2). Clock states are c*b [m]
 * to avoid mixing 1e-9 second clock changes with metre-scale pose changes.
 * All physics and the four original Jacobians come from the official factor.
 */
class PlanarDopplerFactor final
  : public gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Vector3, double, double>
{
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Vector3, double, double>;
public:
  PlanarDopplerFactor(gtsam::Key pose, gtsam::Key velocity, gtsam::Key previous_clock,
    gtsam::Key clock, double doppler, double wavelength,
    const gtsam::Point3 & satellite_position, const gtsam::Point3 & satellite_velocity,
    const gtsam::Point3 & receiver_position, const gtsam::Point3 & lever_arm,
    const gtsam::Pose3 & ecef_from_map, const gtsam::Point3 & angular_velocity,
    double dt, double satellite_clock_drift, const gtsam::SharedNoiseModel & noise)
  : Base(noise, pose, velocity, previous_clock, clock),
    official_(pose, velocity, previous_clock, clock, doppler, wavelength,
      satellite_position, satellite_velocity, receiver_position, lever_arm,
      ecef_from_map, angular_velocity, dt, satellite_clock_drift, noise) {}

  gtsam::Vector evaluateError(const gtsam::Pose2 & pose, const gtsam::Vector3 & velocity,
    const double & previous_clock_m, const double & clock_m,
    gtsam::OptionalMatrixType Hpose = nullptr, gtsam::OptionalMatrixType Hvelocity = nullptr,
    gtsam::OptionalMatrixType Hprevious = nullptr,
    gtsam::OptionalMatrixType Hclock = nullptr) const override
  {
    const gtsam::Pose3 lifted(gtsam::Rot3::Rz(pose.theta()),
      gtsam::Point3(pose.x(), pose.y(), 0.0));
    gtsam::Matrix H3, Hp, Hc;
    const auto error = official_.evaluateError(lifted, velocity,
      previous_clock_m / kLightSpeed, clock_m / kLightSpeed,
      Hpose ? &H3 : nullptr, Hvelocity, Hprevious ? &Hp : nullptr, Hclock ? &Hc : nullptr);
    if (Hpose) {
      // Pose2 tangent [body dx, body dy, dyaw] -> Pose3 [rotation, translation].
      Eigen::Matrix<double, 6, 3> lift = Eigen::Matrix<double, 6, 3>::Zero();
      lift(3, 0) = 1.0; lift(4, 1) = 1.0; lift(2, 2) = 1.0;
      *Hpose = H3 * lift;
    }
    if (Hprevious) {*Hprevious = Hp / kLightSpeed;}
    if (Hclock) {*Hclock = Hc / kLightSpeed;}
    return error;
  }
private:
  gtsam::DopplerFactorArm official_;
};

/** Wheel forward speed plus soft no-slip lateral/vertical constraints.
 * This is the explicit attitude/velocity coupling; the Doppler factor alone
 * must not be interpreted as independently observing attitude.
 */
class BodyVelocityFactor final
  : public gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Vector3>
{
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Vector3>;
public:
  BodyVelocityFactor(gtsam::Key pose, gtsam::Key velocity, double speed,
    const gtsam::SharedNoiseModel & noise)
  : Base(noise, pose, velocity), speed_(speed) {}

  gtsam::Vector evaluateError(const gtsam::Pose2 & pose, const gtsam::Vector3 & velocity,
    gtsam::OptionalMatrixType Hpose = nullptr,
    gtsam::OptionalMatrixType Hvelocity = nullptr) const override
  {
    const double c = std::cos(pose.theta()), s = std::sin(pose.theta());
    gtsam::Matrix3 body_from_map;
    body_from_map << c, s, 0, -s, c, 0, 0, 0, 1;
    const gtsam::Vector3 body = body_from_map * velocity;
    if (Hpose) {
      *Hpose = gtsam::Matrix3::Zero();
      (*Hpose)(0, 2) = body.y(); (*Hpose)(1, 2) = -body.x();
    }
    if (Hvelocity) {*Hvelocity = body_from_map;}
    return body - gtsam::Vector3(speed_, 0.0, 0.0);
  }
private:
  double speed_;
};

/** Karto's additive source-frame [dx,dy,wrapped dyaw] error and covariance.
 * Avoid changing the residual/covariance convention to an SE(2) Logmap.
 */
class KartoBetweenFactor final
  : public gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Pose2>
{
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Pose2>;
public:
  KartoBetweenFactor(gtsam::Key first, gtsam::Key second,
    const gtsam::Pose2 & measurement, const gtsam::SharedNoiseModel & noise)
  : Base(noise, first, second), measurement_(measurement) {}

  gtsam::Vector evaluateError(const gtsam::Pose2 & first, const gtsam::Pose2 & second,
    gtsam::OptionalMatrixType Hfirst = nullptr,
    gtsam::OptionalMatrixType Hsecond = nullptr) const override
  {
    gtsam::Matrix3 A, B;
    const auto relative = first.between(second, A, B);
    gtsam::Matrix3 chart = gtsam::Matrix3::Identity();
    chart.topLeftCorner<2, 2>() = relative.rotation().matrix();
    if (Hfirst) {*Hfirst = chart * A;}
    if (Hsecond) {*Hsecond = chart * B;}
    const double angle = relative.theta() - measurement_.theta();
    return gtsam::Vector3(relative.x() - measurement_.x(),
      relative.y() - measurement_.y(), std::atan2(std::sin(angle), std::cos(angle)));
  }
private:
  gtsam::Pose2 measurement_;
};
}  // namespace hdbot_gtsam_backend
