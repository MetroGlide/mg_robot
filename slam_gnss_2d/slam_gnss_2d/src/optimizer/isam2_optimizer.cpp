#include "slam_gnss_2d/optimizer/isam2_optimizer.hpp"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PriorFactor.h>
#include <rclcpp/rclcpp.hpp>

namespace slam_gnss_2d {
namespace optimizer {

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
  pending_values_ = gtsam::Values();
  latest_estimate_ = gtsam::Values();

  pending_values_.insert(node_index, gtsam::Pose2(x, y, theta));
  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(pos_sigma, pos_sigma, yaw_sigma));
  pending_graph_.add(
      gtsam::PriorFactor<gtsam::Pose2>(
          node_index, gtsam::Pose2(x, y, theta), prior_noise));
  initialized_ = true;
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
  auto noise = gtsam::noiseModel::Gaussian::Information(information);
  pending_graph_.add(
      gtsam::BetweenFactor<gtsam::Pose2>(
          from_index, to_index, gtsam::Pose2(dx, dy, dyaw), noise));
}

void ISAM2Optimizer::add_gnss_prior(
    int node_index,
    double x,
    double y,
    double sigma_xy,
    [[maybe_unused]] double yaw_variance,
    const std::string& robust_kernel_type,
    double robust_kernel_scale) {
  std::lock_guard<std::mutex> lock(lock_);
  if (!initialized_) {
    return;
  }
  auto base_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector2(sigma_xy, sigma_xy));
  gtsam::SharedNoiseModel robust_noise;
  if (robust_kernel_type == "cauchy") {
    robust_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(robust_kernel_scale), base_noise);
  } else {
    robust_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(robust_kernel_scale), base_noise);
  }
  pending_graph_.add(
      gtsam::PoseTranslationPrior<gtsam::Pose2>(
          node_index, gtsam::Point2(x, y), robust_noise));
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

void ISAM2Optimizer::update() {
  std::lock_guard<std::mutex> lock(lock_);
  if (!initialized_) {
    return;
  }
  if (pending_graph_.size() == 0 && pending_values_.size() == 0) {
    return;
  }
  isam2_->update(pending_graph_, pending_values_);
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

}  // namespace optimizer
}  // namespace slam_gnss_2d
