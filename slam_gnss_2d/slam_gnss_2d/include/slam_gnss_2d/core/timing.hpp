#pragma once

#include <chrono>

namespace slam_gnss_2d {
namespace core {

// since からの経過時間 [s]
inline double elapsed_sec(std::chrono::steady_clock::time_point since) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}

}  // namespace core
}  // namespace slam_gnss_2d
