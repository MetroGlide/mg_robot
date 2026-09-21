#include "slam_gnss_2d/optimizer/isam2_optimizer.hpp"

#include <algorithm>
#include <cstdio>
#include <string>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <rclcpp/rclcpp.hpp>

namespace slam_gnss_2d {
namespace optimizer {

namespace {

// base にロバストカーネル ("huber" | "cauchy") を適用したノイズモデルを返す。それ以外の kernel は base のまま
gtsam::SharedNoiseModel robustify(
    const gtsam::SharedNoiseModel& base, const std::string& kernel, double scale) {
  if (kernel == "huber") {
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(scale), base);
  }
  if (kernel == "cauchy") {
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(scale), base);
  }
  return base;
}

// GNSS アンテナ位置の観測ファクター。アンテナはロボット座標系で lever_arm の位置にあるので、
// 予測値は pose.transformFrom(lever_arm) (= 位置 + R(yaw) * lever_arm) となる。
// 旋回すると、アンテナ位置はロボット位置を中心に円を描くため、方位にも拘束がかかる。
class LeverArmPositionFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2> {
 public:
  LeverArmPositionFactor(
      gtsam::Key key,
      const gtsam::Point2& measured,
      const gtsam::Point2& lever_arm,
      const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose2>(model, key),
        measured_(measured),
        lever_arm_(lever_arm) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose2& pose,
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    if (H) {
      gtsam::Matrix23 jacobian;
      const gtsam::Point2 predicted = pose.transformFrom(lever_arm_, jacobian);
      *H = jacobian;
      return predicted - measured_;
    }
    return pose.transformFrom(lever_arm_) - measured_;
  }

 private:
  gtsam::Point2 measured_;
  gtsam::Point2 lever_arm_;
};

}  // namespace

ISAM2Optimizer::ISAM2Optimizer(double relinearize_threshold) {
  params_.setRelinearizeThreshold(relinearize_threshold);
  isam2_ = std::make_unique<gtsam::ISAM2>(params_);
}

void ISAM2Optimizer::initialize(
    int node_index,
    double x,
    double y,
    double theta,
    double pos_sigma,
    double yaw_sigma) {
  std::lock_guard<std::mutex> lock(lock_);
  isam2_ = std::make_unique<gtsam::ISAM2>(params_);
  pending_graph_ = gtsam::NonlinearFactorGraph();
  geom_graph_ = gtsam::NonlinearFactorGraph();
  gnss_graph_ = gtsam::NonlinearFactorGraph();
  pending_values_ = gtsam::Values();
  latest_estimate_ = gtsam::Values();

  pending_values_.insert(node_index, gtsam::Pose2(x, y, theta));
  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(pos_sigma, pos_sigma, yaw_sigma));
  auto prior_factor = gtsam::PriorFactor<gtsam::Pose2>(
      node_index, gtsam::Pose2(x, y, theta), prior_noise);
  pending_graph_.add(prior_factor);
  geom_graph_.add(prior_factor);
  initialized_ = true;
}

void ISAM2Optimizer::set_between_robust_kernel(const std::string& kernel, double scale) {
  std::lock_guard<std::mutex> lock(lock_);
  between_robust_kernel_ = kernel;
  between_robust_scale_ = scale;
}

void ISAM2Optimizer::add_between_factor(
    int from_index,
    int to_index,
    double dx,
    double dy,
    double dyaw,
    const Eigen::Matrix3d& information) {
  std::lock_guard<std::mutex> lock(lock_);
  if (!initialized_) {
    return;
  }
  const gtsam::SharedNoiseModel noise = robustify(
      gtsam::noiseModel::Gaussian::Information(information),
      between_robust_kernel_, between_robust_scale_);
  auto factor = gtsam::BetweenFactor<gtsam::Pose2>(
      from_index, to_index, gtsam::Pose2(dx, dy, dyaw), noise);
  pending_graph_.add(factor);
  geom_graph_.add(factor);
}

void ISAM2Optimizer::add_gnss_prior(
    int node_index,
    double x,
    double y,
    double sigma_xy,
    [[maybe_unused]] double yaw_variance,
    const std::string& robust_kernel_type,
    double robust_kernel_scale,
    const gtsam::Point2& lever_arm) {
  std::lock_guard<std::mutex> lock(lock_);
  if (!initialized_) {
    return;
  }
  // カーネルは huber / cauchy のみ。cauchy 以外は huber として扱う
  const gtsam::SharedNoiseModel noise = robustify(
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(sigma_xy, sigma_xy)),
      robust_kernel_type == "cauchy" ? "cauchy" : "huber", robust_kernel_scale);
  const auto factor = LeverArmPositionFactor(node_index, gtsam::Point2(x, y), lever_arm, noise);
  pending_graph_.add(factor);
  gnss_graph_.add(factor);
}

void ISAM2Optimizer::add_initial_estimate(
    int node_index,
    double x,
    double y,
    double theta) {
  std::lock_guard<std::mutex> lock(lock_);
  if (!initialized_) {
    return;
  }
  if (pending_values_.exists(node_index)) {
    return;
  }
  pending_values_.insert(node_index, gtsam::Pose2(x, y, theta));
}

void ISAM2Optimizer::log_indeterminant_variable(gtsam::Key key) const {
  // 失敗した変数に接続するファクターと、現在の線形化点での誤差を出力する (原因の切り分け用)
  const auto logger = rclcpp::get_logger("slam_gnss_2d.isam2_optimizer");
  const gtsam::Values linearization_point = isam2_->getLinearizationPoint();
  RCLCPP_ERROR(logger, "Indeterminant linear system near variable %zu; connected factors:", static_cast<size_t>(key));
  for (const auto& factor : isam2_->getFactorsUnsafe()) {
    if (!factor || std::find(factor->begin(), factor->end(), key) == factor->end()) {
      continue;
    }
    std::string keys;
    for (const auto k : factor->keys()) {
      keys += std::to_string(k);
      if (linearization_point.exists(k)) {
        const auto pose = linearization_point.at<gtsam::Pose2>(k);
        char buf[96];
        std::snprintf(buf, sizeof(buf), "(%.4g,%.4g,%.4g)", pose.x(), pose.y(), pose.theta());
        keys += buf;
      }
      keys += " ";
    }
    RCLCPP_ERROR(
        logger, "  factor keys=[%s] error=%.6g", keys.c_str(), factor->error(linearization_point));
  }
}

void ISAM2Optimizer::update() {
  std::lock_guard<std::mutex> lock(lock_);
  if (!initialized_) {
    return;
  }
  if (pending_graph_.size() == 0 && pending_values_.size() == 0) {
    return;
  }
  try {
    isam2_->update(pending_graph_, pending_values_);
  } catch (const gtsam::IndeterminantLinearSystemException& e) {
    log_indeterminant_variable(e.nearbyVariable());
    throw;
  }
  latest_estimate_ = isam2_->calculateEstimate();
  pending_graph_ = gtsam::NonlinearFactorGraph();
  pending_values_ = gtsam::Values();
}

std::optional<std::tuple<double, double, double>> ISAM2Optimizer::get_pose(int node_index) const {
  std::lock_guard<std::mutex> lock(lock_);
  if (!initialized_) {
    return std::nullopt;
  }
  if (!latest_estimate_.exists(node_index)) {
    return std::nullopt;
  }
  const auto& pose = latest_estimate_.at<gtsam::Pose2>(node_index);
  return std::make_tuple(pose.x(), pose.y(), pose.theta());
}

std::unordered_map<int, std::tuple<double, double, double>> ISAM2Optimizer::get_all_poses() const {
  std::lock_guard<std::mutex> lock(lock_);
  if (!initialized_) {
    return {};
  }
  std::unordered_map<int, std::tuple<double, double, double>> out;
  for (const auto& key_value : latest_estimate_) {
    const auto& pose = key_value.value.cast<gtsam::Pose2>();
    out[static_cast<int>(key_value.key)] = std::make_tuple(pose.x(), pose.y(), pose.theta());
  }
  return out;
}

void ISAM2Optimizer::run_batch_optimization(int max_iterations) {
  std::lock_guard<std::mutex> lock(lock_);
  if (!initialized_ || geom_graph_.empty()) {
    return;
  }
  if (pending_graph_.size() > 0 || pending_values_.size() > 0) {
    isam2_->update(pending_graph_, pending_values_);
    latest_estimate_ = isam2_->calculateEstimate();
    pending_graph_ = gtsam::NonlinearFactorGraph();
    pending_values_ = gtsam::Values();
  }

  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.maxIterations = max_iterations;
  lm_params.setVerbosity("SILENT");

  // Stage 1: LiDAR幾何最適化 (slam_toolbox同等の歪みのない直線美を確定)
  RCLCPP_INFO(
      rclcpp::get_logger("slam_gnss_2d.isam2_optimizer"),
      "Stage 1: Optimizing pure LiDAR geometric graph (%zu factors, %zu values)",
      geom_graph_.size(), latest_estimate_.size());
  try {
    gtsam::LevenbergMarquardtOptimizer geom_optimizer(geom_graph_, latest_estimate_, lm_params);
    latest_estimate_ = geom_optimizer.optimize();
    RCLCPP_INFO(
        rclcpp::get_logger("slam_gnss_2d.isam2_optimizer"),
        "Stage 1 completed. Geometric error: %.4f", geom_optimizer.error());
  } catch (const std::exception& e) {
    RCLCPP_WARN(
        rclcpp::get_logger("slam_gnss_2d.isam2_optimizer"),
        "Stage 1 optimization exception: %s", e.what());
  }

  // Stage 2: 幾何剛性を保持したまま UTM 座標系への大域アライメント
  if (!gnss_graph_.empty()) {
    RCLCPP_INFO(
        rclcpp::get_logger("slam_gnss_2d.isam2_optimizer"),
        "Stage 2: Aligning to UTM with %zu GNSS priors", gnss_graph_.size());
    try {
      gtsam::NonlinearFactorGraph combined_graph = geom_graph_;
      for (const auto& factor : gnss_graph_) {
        combined_graph.add(factor);
      }
      gtsam::LevenbergMarquardtOptimizer utm_optimizer(combined_graph, latest_estimate_, lm_params);
      latest_estimate_ = utm_optimizer.optimize();
      RCLCPP_INFO(
          rclcpp::get_logger("slam_gnss_2d.isam2_optimizer"),
          "Stage 2 completed. Final UTM-aligned error: %.4f", utm_optimizer.error());
    } catch (const std::exception& e) {
      RCLCPP_WARN(
          rclcpp::get_logger("slam_gnss_2d.isam2_optimizer"),
          "Stage 2 optimization exception: %s", e.what());
    }
  }
}

}  // namespace optimizer
}  // namespace slam_gnss_2d
