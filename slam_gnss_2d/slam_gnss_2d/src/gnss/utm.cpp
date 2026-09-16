#include "slam_gnss_2d/gnss/utm.hpp"

#include <GeographicLib/UTMUPS.hpp>

namespace slam_gnss_2d {
namespace gnss {

UtmTransformer build_utm_transformer(double latitude, double longitude) {
  int zone = 0;
  bool northp = true;
  double x = 0.0;
  double y = 0.0;
  GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, x, y);
  return UtmTransformer{zone, northp};
}

std::pair<double, double> transform_latlon(
    const UtmTransformer& transformer, double latitude, double longitude) {
  int zone = transformer.zone;
  bool northp = transformer.northp;
  double x = 0.0;
  double y = 0.0;
  GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, x, y, transformer.zone);
  return {x, y};
}

}  // namespace gnss
}  // namespace slam_gnss_2d
