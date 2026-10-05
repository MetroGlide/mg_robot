#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <slam_gnss_2d_msgs/srv/save_slam_map.hpp>

namespace {

void print_help(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " [OPTIONS]\n\n"
            << "Options:\n"
            << "  -d, --dir <PATH>        Target output directory (default: current working directory)\n"
            << "  -m, --map-topic <TOPIC> Topic name for OccupancyGrid (default: /map)\n"
            << "  -s, --service <NAME>    Save service name (default: /slam_gnss_2d/save_slam_map)\n"
            << "  --no-pgm                Skip saving map.pgm and map.yaml\n"
            << "  -t, --timeout <SEC>     Timeout in seconds (default: 5.0)\n"
            << "  -h, --help              Show this help message and exit\n";
}

bool save_occupancy_grid_as_pgm(
    const nav_msgs::msg::OccupancyGrid& grid,
    const std::string& output_dir) {
  std::filesystem::create_directories(output_dir);
  std::string pgm_path = (std::filesystem::path(output_dir) / "map.pgm").string();
  std::string yaml_path = (std::filesystem::path(output_dir) / "map.yaml").string();

  int width = static_cast<int>(grid.info.width);
  int height = static_cast<int>(grid.info.height);
  if (width <= 0 || height <= 0 || grid.data.empty()) {
    std::cerr << "Error: Received empty or invalid OccupancyGrid data.\n";
    return false;
  }

  std::ofstream pgm(pgm_path, std::ios::binary);
  if (!pgm.is_open()) {
    std::cerr << "Error: Failed to open " << pgm_path << " for writing.\n";
    return false;
  }

  pgm << "P5\n" << width << " " << height << "\n255\n";
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      int8_t val = grid.data[y * width + x];
      uint8_t pixel;
      if (val == 0) {
        pixel = 254;  // Free space
      } else if (val == 100) {
        pixel = 0;    // Occupied
      } else if (val < 0) {
        pixel = 205;  // Unknown
      } else if (val >= 65) {
        pixel = 0;
      } else if (val <= 25) {
        pixel = 254;
      } else {
        pixel = 205;
      }
      pgm.put(static_cast<char>(pixel));
    }
  }
  pgm.close();

  std::ofstream yf(yaml_path);
  if (!yf.is_open()) {
    std::cerr << "Error: Failed to open " << yaml_path << " for writing.\n";
    return false;
  }

  yf << "image: map.pgm\n"
     << "mode: trinary\n"
     << "resolution: " << grid.info.resolution << "\n"
     << "origin: ["
     << std::fixed << std::setprecision(6)
     << grid.info.origin.position.x << ", "
     << grid.info.origin.position.y << ", 0.0]\n"
     << "negate: 0\n"
     << "occupied_thresh: 0.65\n"
     << "free_thresh: 0.25\n";
  yf.close();

  std::cout << "[save_slam_map_cli] OccupancyGrid saved: " << pgm_path << " and " << yaml_path << "\n";
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::string target_dir;
  std::string map_topic = "/map";
  std::string service_name = "/slam_gnss_2d/save_slam_map";
  bool save_pgm = true;
  double timeout_sec = 5.0;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      print_help(argv[0]);
      return 0;
    } else if ((arg == "-d" || arg == "--dir") && i + 1 < argc) {
      target_dir = argv[++i];
    } else if ((arg == "-m" || arg == "--map-topic") && i + 1 < argc) {
      map_topic = argv[++i];
    } else if ((arg == "-s" || arg == "--service") && i + 1 < argc) {
      service_name = argv[++i];
    } else if (arg == "--no-pgm") {
      save_pgm = false;
    } else if ((arg == "-t" || arg == "--timeout") && i + 1 < argc) {
      timeout_sec = std::stod(argv[++i]);
    } else {
      std::cerr << "Unknown option: " << arg << "\n";
      print_help(argv[0]);
      return 1;
    }
  }

  std::filesystem::path p;
  if (target_dir.empty()) {
    p = std::filesystem::current_path();
  } else {
    p = std::filesystem::path(target_dir);
    if (p.is_relative()) {
      p = std::filesystem::absolute(p);
    }
  }
  std::string resolved_dir = p.lexically_normal().string();
  std::cout << "[save_slam_map_cli] Target output directory: " << resolved_dir << "\n";

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("save_slam_map_cli");

  auto client = node->create_client<slam_gnss_2d_msgs::srv::SaveSlamMap>(service_name);
  auto timeout = std::chrono::duration<double>(timeout_sec);

  std::cout << "[save_slam_map_cli] Waiting for service " << service_name << "...\n";
  if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::milliseconds>(timeout))) {
    std::cerr << "Error: Service " << service_name << " is not available.\n";
    rclcpp::shutdown();
    return 1;
  }

  auto req = std::make_shared<slam_gnss_2d_msgs::srv::SaveSlamMap::Request>();
  req->map_dir = resolved_dir;

  auto future = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, future, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
    std::cerr << "Error: Call to service " << service_name << " failed or timed out.\n";
    rclcpp::shutdown();
    return 1;
  }

  auto resp = future.get();
  if (!resp->success) {
    std::cerr << "Error from SLAM service: " << resp->message << "\n";
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "[save_slam_map_cli] " << resp->message << "\n";

  if (save_pgm) {
    std::cout << "[save_slam_map_cli] Waiting for map message on " << map_topic << "...\n";
    nav_msgs::msg::OccupancyGrid::SharedPtr received_grid = nullptr;

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, map_qos,
        [&received_grid](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          received_grid = msg;
        });

    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !received_grid) {
      rclcpp::spin_some(node);
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() > timeout_sec) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (received_grid) {
      if (!save_occupancy_grid_as_pgm(*received_grid, resolved_dir)) {
        std::cerr << "Warning: Failed to save OccupancyGrid as PGM.\n";
      }
    } else {
      std::cerr << "Warning: Timed out waiting for OccupancyGrid on topic " << map_topic
                << ". PGM/YAML map was not saved.\n";
    }
  }

  std::cout << "[save_slam_map_cli] All operations complete.\n";
  rclcpp::shutdown();
  return 0;
}
