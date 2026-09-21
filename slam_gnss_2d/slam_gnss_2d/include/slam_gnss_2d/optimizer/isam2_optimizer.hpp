#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>

namespace slam_gnss_2d {
namespace optimizer {

class ISAM2Optimizer {
 public:
  explicit ISAM2Optimizer(double relinearize_threshold = 0.1);

  void initialize(
      int node_index,
      double x,
      double y,
      double theta,
      double pos_sigma,
      double yaw_sigma);

  void add_between_factor(
      int from_index,
      int to_index,
      double dx,
      double dy,
      double dyaw,
      const Eigen::Matrix3d& information);

  void add_gnss_prior(
      int node_index,
      double x,
      double y,
      double sigma_xy,
      double yaw_variance = 0.0,
      const std::string& robust_kernel_type = "huber",
      double robust_kernel_scale = 1.345,
      double lever_arm_x = 0.0,
      double lever_arm_y = 0.0);

  void add_initial_estimate(
      int node_index,
      double x,
      double y,
      double theta);

  void update();

  std::optional<std::tuple<double, double, double>> get_pose(int node_index) const;

  std::unordered_map<int, std::tuple<double, double, double>> get_all_poses() const;

  void run_batch_optimization(int max_iterations = 100);

  // between factor (スキャンマッチングのエッジ) のロバスト化。kernel: "none" | "huber" | "cauchy"
  // scale は白色化した残差に対するしきい値 (標準偏差の何倍か)。以降に追加する factor に適用される
  void set_between_robust_kernel(const std::string& kernel, double scale);

 private:
  void log_indeterminant_variable(gtsam::Key key) const;

  gtsam::ISAM2Params params_;
  std::unique_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph pending_graph_;
  gtsam::NonlinearFactorGraph geom_graph_;
  gtsam::NonlinearFactorGraph gnss_graph_;
  gtsam::Values pending_values_;
  gtsam::Values latest_estimate_;
  bool initialized_{false};
  std::string between_robust_kernel_{"none"};
  double between_robust_scale_{1.345};
  mutable std::mutex lock_;
};

}  // namespace optimizer
}  // namespace slam_gnss_2d
