// Copyright 2026 hd
// SPDX-License-Identifier: Apache-2.0

#include "hdbot_gtsam_backend/graph_backend.hpp"
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <set>

namespace hdbot_gtsam_backend
{
void GraphBackend::reset()
{
  nodes.clear(); edges.clear(); values_.clear(); reference_poses_.clear();
}

void GraphBackend::modifyNode(int id, const gtsam::Pose2 & pose)
{
  nodes.at(id).pose = pose; //第一行修改节点当前的位姿；.at(id) 要求节点已经存在，如果找不到这个编号，会抛出异常。
  reference_poses_[id] = pose; //第二行更新它的参考位姿。
}

SolveReport GraphBackend::solve(const std::vector<MatchedEpoch> & input) //准备一次优化所需的因子图和初值。
{
  using gtsam::Symbol;
  SolveReport report;
  if (nodes.empty()) {report.message = "empty graph"; return report;}
  try {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    for (auto it = reference_poses_.begin(); it != reference_poses_.end();) 
    { //删除已经不存在的节点所对应的参考位姿，避免留下过期记录。
      if (!nodes.count(it->first)) {it = reference_poses_.erase(it);} else {++it;}
    }
    std::map<int, std::vector<int>> neighbors;//保存节点编号及对应的邻居列表。
    for (const auto & [id, node] : nodes) {
      initial.insert(Symbol('x', id), node.pose);//加入位姿初值。
      reference_poses_.try_emplace(id, node.pose);//如果没有参考位姿，就记录当前位姿；已有参考位姿时保留原值。
      neighbors[id];//给节点创建邻接表条目，即使它没有任何边。
    }
    for (const auto & edge : edges) {
      if (nodes.count(edge.first) && nodes.count(edge.second)) {
        neighbors[edge.first].push_back(edge.second);
        neighbors[edge.second].push_back(edge.first);
      }
    }
    // 删除节点或边可能使图分裂；为每个连通分量添加参考先验，
    // 消除整体平移和旋转的自由度，避免优化阻尼掩盖欠约束问题。

    std::set<int> visited; //定义一个名为 visited 的集合，用于保存整数，并且集合中的元素不会重复
    for (const auto & [root, adjacent] : neighbors) {
      (void)adjacent;
      if (visited.count(root)) {continue;}
      //emplace_shared用来创建因子
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose2>>(
        Symbol('x', root),// 约束哪个位姿
        reference_poses_.at(root),// 希望它接近哪个参考值
       gtsam::noiseModel::Diagonal::Sigmas(options.anchor_sigmas));
      std::vector<int> stack{root}; //用这个数组模拟一个后进先出的栈，保存接下来要访问的节点：
      while (!stack.empty()) {
        const int current = stack.back(); stack.pop_back();
        if (!visited.insert(current).second) {continue;}
        for (int next : neighbors.at(current)) {stack.push_back(next);}
      }
    }
// 遍历位姿图中的所有边
    for (const auto & edge : edges) {

        // 检查这条边连接的两个节点是否都存在
        // 如果有任何一个节点不存在，则跳过这条边
        if (!nodes.count(edge.first) || !nodes.count(edge.second)) {
            continue;
        }

        // 创建一个 KartoBetweenFactor 相对位姿因子
        // 并将其加入 GTSAM 非线性因子图
        graph.emplace_shared<KartoBetweenFactor>(
            Symbol('x', edge.first),   // 起始节点的变量键
            Symbol('x', edge.second),  // 目标节点的变量键
            edge.measurement,          // 两个节点之间的相对位姿测量值
            gtsam::noiseModel::Gaussian::Covariance(edge.covariance)
                                      // 根据测量协方差创建高斯噪声模型
        );

        // 成功添加一个相对位姿因子，计数加1
        ++report.pose_edges;
    }
    auto epochs = input;//再按 time 升序排列，所以调用者传入的 input 不会被排序修改。
    std::sort(epochs.begin(), epochs.end(), [](const auto & a, const auto & b) {
      return a.time < b.time;
    });

    std::vector<std::shared_ptr<PlanarDopplerFactor>> doppler;
    size_t clock_index = 0, velocity_index = 0;
    double previous_time = 0;
    bool first = true;
    for (const auto & epoch : epochs) {
      //找不到对应位姿节点，或者没有卫星观测，就跳过。
      //. 检查对应的 SLAM 位姿节点是否存在   检查卫星观测是否为空
      if (!nodes.count(epoch.pose_id) || epoch.satellites.empty()) {continue;}
      // 每段首历元只锚定前一时刻钟差；后续历元使用真实的正时间间隔构造因子。
 
      if (first || epoch.time - previous_time > options.max_epoch_gap) {
        if (!first) {++clock_index;}
        initial.insert(Symbol('c', clock_index), 0.0);
        graph.emplace_shared<gtsam::PriorFactor<double>>(Symbol('c', clock_index), 0.0,
          gtsam::noiseModel::Constrained::All(1));//// 添加约束：将这个钟差作为零值参考
        previous_time = epoch.time;
        first = false;
        continue;
      }
      const double dt = epoch.time - previous_time;
      if (!(dt > 0)) {continue;}
      const auto previous_clock = Symbol('c', clock_index);
      const auto clock = Symbol('c', ++clock_index);
      const auto velocity = Symbol('v', velocity_index++);
      const auto pose = Symbol('x', epoch.pose_id);
      initial.insert(clock, 0.0); // 钟差状态以零作为初值，单位为米。
      const auto & X = nodes.at(epoch.pose_id).pose;
      initial.insert(velocity, gtsam::Vector3(epoch.wheel_speed * std::cos(X.theta()),
        epoch.wheel_speed * std::sin(X.theta()), 0.0));
      // 车轮前向速度及无侧滑软约束将地图速度与机器人航向耦合。
      graph.emplace_shared<BodyVelocityFactor>(pose, velocity, epoch.wheel_speed,
        gtsam::noiseModel::Diagonal::Sigmas(options.body_velocity_sigmas));
      for (const auto & sat : epoch.satellites) {
        // Huber 鲁棒核降低大残差观测的影响，但不能替代上游观测质量筛选。
        auto noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(options.huber),
          gtsam::noiseModel::Isotropic::Sigma(1, sat.sigma));
        auto factor = std::make_shared<PlanarDopplerFactor>(pose, velocity,
          previous_clock, clock, sat.doppler, sat.wavelength, sat.position, sat.velocity,
          epoch.receiver_position, options.lever_arm, options.ecef_from_map,
          epoch.angular_velocity, dt, sat.clock_drift, noise);
        graph.push_back(factor); doppler.push_back(factor);
      }
      ++report.epochs;
      previous_time = epoch.time;
    }
    report.doppler_factors = doppler.size();
    report.initial_error = graph.error(initial);
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(50);
    params.setRelativeErrorTol(1e-6);
    const auto optimized = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
    report.final_error = graph.error(optimized);
    if (!std::isfinite(report.final_error) || report.final_error > report.initial_error + 1e-6) {
      throw std::runtime_error("optimizer returned non-finite or increased graph error");
    }
    double squared = 0;
    for (const auto & factor : doppler) {
      squared += factor->unwhitenedError(optimized).squaredNorm();
    }
    report.doppler_rms = doppler.empty() ? 0.0 : std::sqrt(squared / doppler.size());
    // 先验证全部位姿，再统一回写；优化失败时保留上一次有效结果。
    for (const auto & [id, node] : nodes) {
      (void)node;
      const auto p = optimized.at<gtsam::Pose2>(Symbol('x', id));
      if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.theta())) {
        throw std::runtime_error("non-finite optimized pose");
      }
    }
    for (auto & [id, node] : nodes) {node.pose = optimized.at<gtsam::Pose2>(Symbol('x', id));}
    values_ = optimized;
    report.success = true;
  } catch (const std::exception & error) {report.message = error.what();}
  return report;
}
}  // namespace hdbot_gtsam_backend
