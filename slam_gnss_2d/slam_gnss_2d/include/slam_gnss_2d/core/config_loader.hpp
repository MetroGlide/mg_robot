#pragma once

#include <rclcpp/rclcpp.hpp>

#include "slam_gnss_2d/core/config.hpp"

namespace slam_gnss_2d {

class ConfigLoader {
 public:
  static void declare_params(rclcpp::Node& node);
  static SlamConfig build_config(const rclcpp::Node& node);
};

}  // namespace slam_gnss_2d
