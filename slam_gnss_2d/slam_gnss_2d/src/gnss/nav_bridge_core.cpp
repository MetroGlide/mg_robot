#include "slam_gnss_2d/gnss/nav_bridge_core.hpp"

#include <algorithm>
#include <cmath>

namespace slam_gnss_2d {
namespace gnss {

CarrierSolution CarrierFromFlags(std::uint8_t flags) {
  switch ((flags >> 6) & 0x03) {
    case 2:
      return CarrierSolution::kFixed;
    case 1:
      return CarrierSolution::kFloat;
    default:
      return CarrierSolution::kNone;
  }
}

std::optional<double> PositionVariance(
    const PositionQualityConfig& config, CarrierSolution carrier, double h_acc_m) {
  double floor_m = config.single_floor_m;
  double scale = config.single_scale;
  switch (carrier) {
    case CarrierSolution::kFixed:
      floor_m = config.fix_floor_m;
      scale = config.fix_scale;
      break;
    case CarrierSolution::kFloat:
      floor_m = config.float_floor_m;
      scale = config.float_scale;
      break;
    case CarrierSolution::kNone:
      if (!config.accept_single) {
        return std::nullopt;
      }
      break;
  }
  const double sigma = std::max(h_acc_m, floor_m);
  const double variance = sigma * sigma * scale;
  if (variance * 2.0 > config.max_covariance_threshold) {
    return std::nullopt;
  }
  return variance;
}

Point2 AntennaToBase(Point2 antenna, double base_yaw, double lever_x, double lever_y) {
  const double c = std::cos(base_yaw);
  const double s = std::sin(base_yaw);
  return {antenna.x - (c * lever_x - s * lever_y), antenna.y - (s * lever_x + c * lever_y)};
}

double LeverArmUncertaintyVariance(double lever_x, double lever_y) {
  return lever_x * lever_x + lever_y * lever_y;
}

}  // namespace gnss
}  // namespace slam_gnss_2d
