// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#include "hdbot_gtsam_backend/gtsam_solver.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>

namespace hdbot_gtsam_backend
{
namespace
{
template<class T>
T parameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, const T & value)
{
  if (!node->has_parameter(name)) {node->declare_parameter<T>(name, value);}
  return node->get_parameter(name).get_value<T>();
}
gtsam::Vector3 vectorParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, const std::vector<double> & fallback)
{
  const auto a = parameter(node, name, fallback);
  if (a.size() != 3 || !std::all_of(a.begin(), a.end(), [](double v) {return std::isfinite(v);})) {
    throw std::invalid_argument(name + " must contain three finite values");
  }
  return {a[0], a[1], a[2]};
}
gtsam::Rot3 rotationParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name)
{
  const auto q = parameter(node, name, std::vector<double>{0, 0, 0, 1});
  if (q.size() != 4) {throw std::invalid_argument(name + " must be [x,y,z,w]");}
  double norm = 0;
  for (double v : q) {norm += v * v;}
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
    throw std::invalid_argument(name + " must be a unit quaternion");
  }
  const double n = std::sqrt(norm);
  return gtsam::Rot3::Quaternion(q[3]/n, q[0]/n, q[1]/n, q[2]/n);
}
template<class Map>
std::optional<typename Map::mapped_type> interpolate(const Map & buffer, double time, double tolerance)
{
  if (buffer.empty()) {return std::nullopt;}
  // 双侧样本均在容差内才线性插值，避免跨越长时间缺测区间。
  auto after = buffer.lower_bound(time);
  if (after != buffer.end() && after->first == time) {return after->second;}
  if (after != buffer.begin() && after != buffer.end()) {
    const auto before = std::prev(after);
    if (time - before->first <= tolerance && after->first - time <= tolerance) {
      const double alpha = (time - before->first) / (after->first - before->first);
      typename Map::mapped_type result = (1.0 - alpha) * before->second + alpha * after->second;
      return result;
    }
  }
  // 缓冲边界缺少双侧样本时，仅在时间容差内使用最近邻；否则拒绝关联。
  if (after != buffer.end() && std::abs(after->first - time) <= tolerance) {return after->second;}
  if (after != buffer.begin()) {
    --after;
    if (std::abs(after->first - time) <= tolerance) {return after->second;}
  }
  return std::nullopt;
}
template<class V> gtsam::Point3 vector(const V & v) {return {v.x, v.y, v.z};}
template<class Map> void bound(Map & map, size_t limit)
{
  while (map.size() > limit) {map.erase(map.begin());}
}
}  // namespace

void GtsamSolver::Configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  node_ = node;
  enabled_ = parameter(node, "gnss.enabled", true);
  // 未明确完成 map 到 ECEF 的地理标定时，不允许把 GNSS 约束加入图。
  georeference_ready_ = parameter(node, "gnss.georeference_ready", false);
  base_frame_ = parameter(node, "base_frame", std::string("base_footprint"));
  imu_frame_ = parameter(node, "gnss.imu_frame", std::string("imu_link"));
  antenna_frame_ = parameter(node, "gnss.antenna_frame", std::string("gnss_antenna"));
  sync_tolerance_ = parameter(node, "gnss.sensor_sync_tolerance_s", 0.15);
  pose_tolerance_ = parameter(node, "gnss.pose_sync_tolerance_s", 0.15);
  minimum_satellites_ = parameter(node, "gnss.minimum_satellites", 4);
  max_epochs_ = parameter(node, "gnss.max_epochs", 2000);
  backend_.options.lever_arm = vectorParameter(node, "gnss.lever_arm_body_m", {0, 0, 0});
  backend_.options.ecef_from_map = gtsam::Pose3(
    rotationParameter(node, "gnss.ecef_from_map_quaternion_xyzw"),
    vectorParameter(node, "gnss.ecef_from_map_translation_m", {0, 0, 0}));
  body_from_imu_ = rotationParameter(node, "gnss.body_from_imu_quaternion_xyzw");
  gyro_bias_ = vectorParameter(node, "gnss.gyro_bias_imu_radps", {0, 0, 0});
  backend_.options.body_velocity_sigmas = vectorParameter(node,
    "gnss.body_velocity_sigmas_mps", {0.15, 0.15, 0.1});
  backend_.options.anchor_sigmas = vectorParameter(node,
    "gnss.anchor_sigmas_xy_yaw", {0.05, 0.05, 3.14});
  backend_.options.max_epoch_gap = parameter(node, "gnss.max_epoch_gap_s", 3.0);
  backend_.options.huber = parameter(node, "gnss.huber", 1.345);
  if (!(sync_tolerance_ > 0) || !std::isfinite(sync_tolerance_) ||
    !(pose_tolerance_ > 0) || !std::isfinite(pose_tolerance_) || minimum_satellites_ < 4 ||
    max_epochs_ < 2 || !(backend_.options.body_velocity_sigmas.array() > 0).all() ||
    !(backend_.options.anchor_sigmas.array() > 0).all() ||
    !std::isfinite(backend_.options.max_epoch_gap) || !(backend_.options.max_epoch_gap > 0) ||
    !std::isfinite(backend_.options.huber) || !(backend_.options.huber > 0)) {
    throw std::invalid_argument("invalid GNSS synchronization, noise, or buffer parameters");
  }
  if (georeference_ready_ && backend_.options.ecef_from_map.translation().norm() < 6e6) {
    throw std::invalid_argument("georeference_ready requires an Earth-surface ECEF map origin");
  }
  const auto imu_topic = parameter(node, "gnss.imu_topic", std::string("/imu"));
  const auto odom_topic = parameter(node, "gnss.odom_topic", std::string("/odom"));
  const auto epoch_topic = parameter(node, "gnss.doppler_topic", std::string("/gnss/doppler"));
  imu_subscription_ = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic,
    rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      if (msg->header.frame_id != imu_frame_ || msg->angular_velocity_covariance[0] < 0 ||
        msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u) {return;}
      const auto rate = vector(msg->angular_velocity);
      if (!rate.allFinite()) {return;}
      std::lock_guard<std::mutex> lock(mutex_);
      imu_[rclcpp::Time(msg->header.stamp).seconds()] = body_from_imu_.rotate(rate - gyro_bias_);
      bound(imu_, 20000);
      if (matched_cache_.size() < epochs_.size()) {dirty_ = true;}
    });
  odom_subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(odom_topic,
    rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      if (msg->child_frame_id != base_frame_ || !std::isfinite(msg->twist.twist.linear.x) ||
        msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u) {return;}
      std::lock_guard<std::mutex> lock(mutex_);
      wheel_[rclcpp::Time(msg->header.stamp).seconds()] = msg->twist.twist.linear.x;
      bound(wheel_, 20000);
      if (matched_cache_.size() < epochs_.size()) {dirty_ = true;}
    });
  gnss_subscription_ = node->create_subscription<hdbot_gnss_msgs::msg::DopplerEpoch>(
    epoch_topic, rclcpp::SensorDataQoS(),
    [this](hdbot_gnss_msgs::msg::DopplerEpoch::ConstSharedPtr msg) {receiveEpoch(msg);});
  RCLCPP_INFO(node->get_logger(), "GTSAM planar solver; Doppler enabled=%d georeference_ready=%d",
    enabled_, georeference_ready_);
  if (enabled_ && !georeference_ready_) {
    RCLCPP_WARN(node->get_logger(), "GNSS factors wait for configured ECEF-from-map transform; see README");
  }
}

void GtsamSolver::receiveEpoch(hdbot_gnss_msgs::msg::DopplerEpoch::ConstSharedPtr msg)
{
  if (!enabled_ || !georeference_ready_ || msg->header.stamp.sec < 0 ||
    msg->header.stamp.nanosec >= 1000000000u) {return;}
  const auto receiver = vector(msg->receiver_position_ecef);
  if (msg->header.frame_id != antenna_frame_ || !receiver.allFinite() || receiver.norm() < 6e6) {
    if (auto node = node_.lock()) {RCLCPP_WARN(node->get_logger(), "Rejected GNSS frame or nominal ECEF position");}
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  epochs_[rclcpp::Time(msg->header.stamp).seconds()] = *msg;
  matched_cache_.erase(rclcpp::Time(msg->header.stamp).seconds());
  bound(epochs_, static_cast<size_t>(max_epochs_)); dirty_ = true;
  for (auto it = matched_cache_.begin(); it != matched_cache_.end();) {
    if (!epochs_.count(it->first)) {it = matched_cache_.erase(it);} else {++it;}
  }
}

std::vector<MatchedEpoch> GtsamSolver::matchedEpochs()
{
  std::vector<MatchedEpoch> matched;
  if (!enabled_ || !georeference_ready_ || backend_.nodes.empty()) {return matched;}
  for (const auto & [time, msg] : epochs_) {
    const auto cached = matched_cache_.find(time);
    if (cached != matched_cache_.end() && backend_.nodes.count(cached->second.pose_id)) {
      matched.push_back(cached->second);
      continue;
    }
    auto nearest = backend_.nodes.end();
    double distance = std::numeric_limits<double>::infinity();
    for (auto it = backend_.nodes.begin(); it != backend_.nodes.end(); ++it) {
      const double d = std::abs(it->second.time - time);
      if (d < distance) {distance = d; nearest = it;}
    }
    if (distance > pose_tolerance_) {continue;}
    const auto rate = interpolate(imu_, time, sync_tolerance_);
    const auto speed = interpolate(wheel_, time, sync_tolerance_);
    if (!rate || !speed) {continue;}
    MatchedEpoch epoch;
    epoch.time = time; epoch.pose_id = nearest->first;
    epoch.receiver_position = vector(msg.receiver_position_ecef);
    epoch.angular_velocity = *rate; epoch.wheel_speed = *speed;
    std::set<std::string> signals;
    for (const auto & obs : msg.observations) {
      const auto p = vector(obs.satellite_position_ecef), v = vector(obs.satellite_velocity_ecef);
      if (!obs.valid || obs.signal_id.empty() || signals.count(obs.signal_id) ||
        !p.allFinite() || !v.allFinite() || (p - epoch.receiver_position).norm() < 1e6 ||
        !std::isfinite(obs.doppler_hz) || !std::isfinite(obs.wavelength_m) ||
        !(obs.wavelength_m > 0) || !std::isfinite(obs.range_rate_sigma_mps) ||
        !(obs.range_rate_sigma_mps > 0) || !std::isfinite(obs.satellite_clock_drift_sps)) {continue;}
      signals.insert(obs.signal_id);
      epoch.satellites.push_back({obs.signal_id, obs.doppler_hz, obs.wavelength_m,
        obs.satellite_clock_drift_sps, obs.range_rate_sigma_mps, p, v});
    }
    if (epoch.satellites.size() >= static_cast<size_t>(minimum_satellites_)) {
      // Four signals are insufficient if LOS/clock geometry is rank deficient.
      gtsam::Matrix geometry(epoch.satellites.size(), 4);
      for (size_t row = 0; row < epoch.satellites.size(); ++row) {
        const auto & p = epoch.satellites[row].position;
        const gtsam::Vector3 los = (p - epoch.receiver_position).normalized();
        const double k = gtsam::gnss::OMGE / kLightSpeed;
        geometry.block<1, 3>(row, 0) =
          (gtsam::Vector3(-k*p.y(), k*p.x(), 0) - los).transpose();
        geometry(row, 3) = 1.0;
      }
      Eigen::ColPivHouseholderQR<gtsam::Matrix> qr(geometry);
      qr.setThreshold(1e-3);
      if (qr.rank() < 4) {continue;}
      matched_cache_[time] = epoch;
      matched.push_back(std::move(epoch));
    }
  }
  return matched;
}

void GtsamSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan> * vertex)
{
  if (!vertex || !vertex->GetObject()) {return;}
  const auto * scan = vertex->GetObject();
  const auto & pose = scan->GetCorrectedPose();
  std::lock_guard<std::mutex> lock(mutex_);
  backend_.nodes[scan->GetUniqueId()] = {scan->GetTime(), {pose.GetX(), pose.GetY(), pose.GetHeading()}};
  display_graph_[scan->GetUniqueId()] = {pose.GetX(), pose.GetY(), pose.GetHeading()};
  dirty_ = true;
}

void GtsamSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan> * edge)
{
  if (!edge) {return;}
  auto * label = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
  if (!label) {throw std::invalid_argument("Karto constraint has no LinkInfo");}
  PoseEdge converted;
  converted.first = edge->GetSource()->GetObject()->GetUniqueId();
  converted.second = edge->GetTarget()->GetObject()->GetUniqueId();
  const auto & p = label->GetPoseDifference();
  converted.measurement = {p.GetX(), p.GetY(), p.GetHeading()};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {converted.covariance(i, j) = label->GetCovariance()(i, j);}
  }
  if (!converted.covariance.allFinite() ||
    !converted.covariance.isApprox(converted.covariance.transpose(), 1e-8) ||
    Eigen::LLT<gtsam::Matrix3>(converted.covariance).info() != Eigen::Success) {
    throw std::invalid_argument("Karto edge covariance is not symmetric positive definite");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  // Karto has one edge per ordered pair; replace updates rather than double-count.
  auto & edges = backend_.edges;
  const auto it = std::find_if(edges.begin(), edges.end(), [&](const auto & e) {
    return e.first == converted.first && e.second == converted.second;
  });
  if (it == edges.end()) {edges.push_back(converted);} else {*it = converted;}
  dirty_ = true;
}

void GtsamSolver::RemoveNode(kt_int32s id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  backend_.nodes.erase(id); display_graph_.erase(id);
  for (auto it = matched_cache_.begin(); it != matched_cache_.end();) {
    if (it->second.pose_id == id) {it = matched_cache_.erase(it);} else {++it;}
  }
  auto & edges = backend_.edges;
  edges.erase(std::remove_if(edges.begin(), edges.end(), [id](const auto & e) {
    return e.first == id || e.second == id;
  }), edges.end());
  dirty_ = true;
}
void GtsamSolver::RemoveConstraint(kt_int32s first, kt_int32s second)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & edges = backend_.edges;
  edges.erase(std::remove_if(edges.begin(), edges.end(), [=](const auto & e) {
    return (e.first == first && e.second == second) || (e.first == second && e.second == first);
  }), edges.end());
  dirty_ = true;
}
void GtsamSolver::Compute()
{
  std::lock_guard<std::mutex> lock(mutex_);
  corrections_.clear();
  const auto matched = matchedEpochs();
  last_report_ = backend_.solve(matched);
  dirty_ = false;
  if (last_report_.success) {
    for (const auto & [id, node] : backend_.nodes) {
      corrections_.emplace_back(id, karto::Pose2(node.pose.x(), node.pose.y(), node.pose.theta()));
      display_graph_[id] = {node.pose.x(), node.pose.y(), node.pose.theta()};
    }
  }
  if (auto node = node_.lock()) {
    if (!last_report_.success) {
      RCLCPP_ERROR(node->get_logger(), "GTSAM solve failed: %s", last_report_.message.c_str());
    } else {
      RCLCPP_INFO(node->get_logger(),
        "GTSAM edges=%zu Doppler=%zu epochs=%zu error=%.6g->%.6g Doppler_RMS=%.4f m/s",
        last_report_.pose_edges, last_report_.doppler_factors, last_report_.epochs,
        last_report_.initial_error, last_report_.final_error, last_report_.doppler_rms);
      if (epochs_.size() > matched.size()) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 10000,
          "GNSS matched %zu/%zu buffered epochs: check frames, sync tolerances, validity and LOS geometry",
          matched.size(), epochs_.size());
      }
    }
  }
}
void GtsamSolver::Clear() {std::lock_guard<std::mutex> lock(mutex_); corrections_.clear();}
void GtsamSolver::Reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  backend_.reset(); corrections_.clear(); display_graph_.clear();
  imu_.clear(); wheel_.clear(); epochs_.clear(); matched_cache_.clear();
  last_report_ = {}; dirty_ = false;
}
void GtsamSolver::ModifyNode(const int & id, Eigen::Vector3d pose)
{
  if (!pose.allFinite()) {throw std::invalid_argument("non-finite manual pose");}
  std::lock_guard<std::mutex> lock(mutex_);
  backend_.modifyNode(id, {pose.x(), pose.y(), pose.z()});
  display_graph_[id] = pose; dirty_ = true;
}
void GtsamSolver::GetNodeOrientation(const int & id, double & yaw)
{
  std::lock_guard<std::mutex> lock(mutex_); yaw = backend_.nodes.at(id).pose.theta();
}
bool GtsamSolver::pending() const {std::lock_guard<std::mutex> lock(mutex_); return dirty_;}
SolveReport GtsamSolver::lastReport() const
{
  std::lock_guard<std::mutex> lock(mutex_); return last_report_;
}
}  // namespace hdbot_gtsam_backend
PLUGINLIB_EXPORT_CLASS(hdbot_gtsam_backend::GtsamSolver, karto::ScanSolver)
