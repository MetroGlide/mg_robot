#pragma once

#include <optional>
#include <utility>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace gnss {

class GnssAnchorManager {
 public:
  GnssAnchorManager() = default;

  bool is_initialized() const;
  std::optional<std::pair<double, double>> anchor_utm() const;
  std::optional<std::pair<double, double>> anchor_latlon() const;

  bool try_set_anchor(const core::GnssData& gnss, int min_fix_status);
  void set_anchor(double x, double y, double lat, double lon);

  std::pair<double, double> to_local(const core::GnssData& gnss) const;

 private:
  std::optional<double> anchor_x_;
  std::optional<double> anchor_y_;
  std::optional<double> anchor_lat_;
  std::optional<double> anchor_lon_;
};

}  // namespace gnss
}  // namespace slam_gnss_2d
