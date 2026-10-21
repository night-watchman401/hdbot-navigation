// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#include "hdbot_gtsam_backend/gtsam_solver.hpp"
#include "hdbot_gtsam_backend/slam_gnss_node.hpp"
#include <slam_toolbox/slam_toolbox_async.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>

namespace hdbot_gtsam_backend
{
/** 通过 Karto 原有修正流程更新扫描位姿和 map->odom，保持前后端地图一致。 */
class SlamGnssNode : public slam_toolbox::AsynchronousSlamToolbox
{
public:
  explicit SlamGnssNode(const rclcpp::NodeOptions & options)
  : AsynchronousSlamToolbox(options) {}

  slam_toolbox::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    if (!has_parameter("gnss.optimization_period_s")) {
      declare_parameter("gnss.optimization_period_s", 1.0);
    }
    const double period = get_parameter("gnss.optimization_period_s").as_double();
    if (!std::isfinite(period) || period <= 0) {
      throw std::invalid_argument("gnss.optimization_period_s must be positive");
    }
    const auto result = AsynchronousSlamToolbox::on_activate(state);
    if (result != slam_toolbox::CallbackReturn::SUCCESS) {return result;}
    corrected_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("gnss_corrected_pose", 10);
    corrected_pose_->on_activate();
    timer_ = create_wall_timer(std::chrono::duration<double>(period), [this] {correctGraph();});
    return result;
  }
  slam_toolbox::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    timer_.reset();
    if (corrected_pose_) {corrected_pose_->on_deactivate(); corrected_pose_.reset();}
    {
      boost::mutex::scoped_lock lock(smapper_mutex_);
      have_latest_ = false;
    }
    return AsynchronousSlamToolbox::on_deactivate(state);
  }

protected:
  karto::LocalizedRangeScan * addScan(karto::LaserRangeFinder * laser,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan, karto::Pose2 & odom) override
  {
    //返回值是 LocalizedRangeScan 指针，表示处理后带有位姿信息的扫描对象。
    auto * result = AsynchronousSlamToolbox::addScan(laser, scan, odom); 
    if (result) {
      boost::mutex::scoped_lock lock(smapper_mutex_);
      latest_id_ = result->GetUniqueId(); latest_odom_ = odom;
      latest_stamp_ = scan->header.stamp; have_latest_ = true;
    }
    return result;
  }

private:
  void correctGraph()
  {
    auto solver = std::dynamic_pointer_cast<GtsamSolver>(solver_);
    if (!solver || !solver->pending()) {return;}
    // 与激光建图共用互斥锁，避免修正过程中扫描图被并发修改。
    boost::mutex::scoped_lock lock(smapper_mutex_);
    if (!have_latest_) {return;}
    auto * mapper = smapper_->getMapper();
    // 重置、反序列化或交互编辑可能删除扫描；先核对编号和时间戳再应用修正。
    karto::LocalizedRangeScan * latest = nullptr;
    for (auto * scan : mapper->GetAllProcessedScans()) {
      if (scan->GetUniqueId() == latest_id_ &&
        std::abs(scan->GetTime() - rclcpp::Time(latest_stamp_).seconds()) < 1e-6) {latest = scan; break;}
    }
    if (!latest) {have_latest_ = false; return;}
    mapper->CorrectPoses();
    if (!solver->lastReport().success) {return;}
    setTransformFromPoses(latest->GetCorrectedPose(), latest_odom_, latest_stamp_, false);
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = map_frame_; message.header.stamp = latest_stamp_;
    const auto & p = latest->GetCorrectedPose();
    message.pose.position.x = p.GetX(); message.pose.position.y = p.GetY();
    message.pose.orientation.z = std::sin(p.GetHeading() / 2);
    message.pose.orientation.w = std::cos(p.GetHeading() / 2);
    corrected_pose_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr corrected_pose_;
  bool have_latest_{false};
  int latest_id_{};
  karto::Pose2 latest_odom_;
  builtin_interfaces::msg::Time latest_stamp_;
};
std::shared_ptr<rclcpp_lifecycle::LifecycleNode> makeSlamGnssNode(
  const rclcpp::NodeOptions & options)
{
  return std::make_shared<SlamGnssNode>(options);
}
}  // namespace hdbot_gtsam_backend
