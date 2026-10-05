#pragma once

#include <utility>

namespace slam_gnss_2d {
namespace gnss {

struct UtmTransformer {
  int zone{0};
  bool northp{true};
};

UtmTransformer build_utm_transformer(double latitude, double longitude);

std::pair<double, double> transform_latlon(
    const UtmTransformer& transformer, double latitude, double longitude);

}  // namespace gnss
}  // namespace slam_gnss_2d
