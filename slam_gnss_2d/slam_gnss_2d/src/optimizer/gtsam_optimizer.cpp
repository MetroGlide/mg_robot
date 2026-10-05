#include "slam_gnss_2d/optimizer/gtsam_optimizer.hpp"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PriorFactor.h>

#include <rclcpp/rclcpp.hpp>

namespace slam_gnss_2d {
namespace optimizer {

namespace {
const gtsam::Vector3 kAnchorVariances(1e-6, 1e-6, 1e-8);
}

std::vector<core::PoseNode> GTSAMOptimizer::optimize(
    const std::vector<core::PoseNode>& nodes,
    const std::vector<core::PoseEdge>& edges,
    const std::vector<core::GnssPrior>& gnss_priors) {
  if (nodes.size() < 2) {
    return nodes;
  }
  if (edges.empty() && gnss_priors.empty()) {
    return nodes;
  }

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  for (const auto& node : nodes) {
    initial.insert(node.index, gtsam::Pose2(node.x, node.y, node.yaw));
  }

  const auto& anchor = nodes.front();
  auto prior_noise = gtsam::noiseModel::Diagonal::Variances(kAnchorVariances);
  graph.add(
      gtsam::PriorFactor<gtsam::Pose2>(
          anchor.index,
          gtsam::Pose2(anchor.x, anchor.y, anchor.yaw),
          prior_noise));

  for (const auto& edge : edges) {
    auto noise = gtsam::noiseModel::Gaussian::Information(edge.information);
    graph.add(
        gtsam::BetweenFactor<gtsam::Pose2>(
            edge.from_index,
            edge.to_index,
            gtsam::Pose2(edge.dx, edge.dy, edge.dyaw),
            noise));
  }

  for (const auto& gnss_prior : gnss_priors) {
    if (!initial.exists(gnss_prior.node_index)) {
      RCLCPP_WARN(
          rclcpp::get_logger("slam_gnss_2d.gtsam_optimizer"),
          "GNSS prior skipped: node_index=%d not in graph",
          gnss_prior.node_index);
      continue;
    }
    auto gnss_noise = gtsam::noiseModel::Gaussian::Information(gnss_prior.information);
    graph.add(
        gtsam::PoseTranslationPrior<gtsam::Pose2>(
            gnss_prior.node_index,
            gtsam::Point2(gnss_prior.x, gnss_prior.y),
            gnss_noise));
  }

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("SILENT");
  gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();

  std::vector<core::PoseNode> updated;
  updated.reserve(nodes.size());
  for (const auto& node : nodes) {
    const auto& pose = result.at<gtsam::Pose2>(node.index);
    updated.emplace_back(core::PoseNode{
        node.index,
        node.timestamp,
        pose.x(),
        pose.y(),
        pose.theta(),
        node.scan,
    });
  }

  RCLCPP_INFO(
      rclcpp::get_logger("slam_gnss_2d.gtsam_optimizer"),
      "GTSAMOptimizer: %zu nodes, %zu edges, %zu gnss_priors optimized",
      nodes.size(), edges.size(), gnss_priors.size());

  return updated;
}

}  // namespace optimizer
}  // namespace slam_gnss_2d
