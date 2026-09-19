#include "slam_gnss_2d/gnss/anchor_manager.hpp"

#include <GeographicLib/UTMUPS.hpp>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <stdexcept>

namespace slam_gnss_2d {
namespace gnss {

bool GnssAnchorManager::is_initialized() const {
  return anchor_x_.has_value() && anchor_y_.has_value();
}

std::optional<std::pair<double, double>> GnssAnchorManager::anchor_utm() const {
  if (!is_initialized()) {
    return std::nullopt;
  }
  return std::make_pair(*anchor_x_, *anchor_y_);
}

std::optional<std::pair<double, double>> GnssAnchorManager::anchor_latlon() const {
  if (!is_initialized()) {
    return std::nullopt;
  }
  return std::make_pair(*anchor_lat_, *anchor_lon_);
}

bool GnssAnchorManager::try_set_anchor(const core::GnssData& gnss, int min_fix_status) {
  if (is_initialized()) {
    return false;
  }
  if (gnss.fix_status < min_fix_status) {
    RCLCPP_WARN(
        rclcpp::get_logger("slam_gnss_2d.gnss_anchor_manager"),
        "Anchor rejected: fix_status %d < min %d", gnss.fix_status, min_fix_status);
    return false;
  }

  anchor_x_ = gnss.x;
  anchor_y_ = gnss.y;
  anchor_lat_ = gnss.latitude;
  anchor_lon_ = gnss.longitude;

  int zone = 0;
  bool northp = true;
  double dummy_x = 0.0, dummy_y = 0.0;
  GeographicLib::UTMUPS::Forward(gnss.latitude, gnss.longitude, zone, northp, dummy_x, dummy_y);
  utm_zone_ = zone;
  utm_northp_ = northp;

  RCLCPP_INFO(
      rclcpp::get_logger("slam_gnss_2d.gnss_anchor_manager"),
      "Anchor set at UTM E=%.3f, N=%.3f, Lat=%.7f, Lon=%.7f (zone=%d north=%d)",
      gnss.x, gnss.y, gnss.latitude, gnss.longitude, zone, northp);
  return true;
}

void GnssAnchorManager::set_anchor(double x, double y, double lat, double lon) {
  anchor_x_ = x;
  anchor_y_ = y;
  anchor_lat_ = lat;
  anchor_lon_ = lon;

  int zone = 0;
  bool northp = true;
  double dummy_x = 0.0, dummy_y = 0.0;
  GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, dummy_x, dummy_y);
  utm_zone_ = zone;
  utm_northp_ = northp;
}

void GnssAnchorManager::update_anchor(double x, double y) {
  anchor_x_ = x;
  anchor_y_ = y;
  if (utm_zone_.has_value() && utm_northp_.has_value()) {
    double lat = 0.0, lon = 0.0;
    try {
      GeographicLib::UTMUPS::Reverse(*utm_zone_, *utm_northp_, x, y, lat, lon);
      anchor_lat_ = lat;
      anchor_lon_ = lon;
      RCLCPP_INFO(
          rclcpp::get_logger("slam_gnss_2d.gnss_anchor_manager"),
          "Anchor UTM updated to E=%.3f, N=%.3f, Lat=%.7f, Lon=%.7f", x, y, lat, lon);
      return;
    } catch (const std::exception& e) {
      RCLCPP_WARN(
          rclcpp::get_logger("slam_gnss_2d.gnss_anchor_manager"),
          "Failed to reverse UTM to Lat/Lon: %s", e.what());
    }
  }
  RCLCPP_INFO(
      rclcpp::get_logger("slam_gnss_2d.gnss_anchor_manager"),
      "Anchor UTM updated to E=%.3f, N=%.3f", x, y);
}

std::pair<double, double> GnssAnchorManager::to_local(const core::GnssData& gnss) const {
  if (!is_initialized()) {
    throw std::runtime_error("GNSS anchor is not initialized");
  }
  return {gnss.x - *anchor_x_, gnss.y - *anchor_y_};
}

}  // namespace gnss
}  // namespace slam_gnss_2d
