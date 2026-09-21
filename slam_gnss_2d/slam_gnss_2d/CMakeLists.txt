cmake_minimum_required(VERSION 3.8)
project(slam_gnss_2d)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(CMAKE_BUILD_TYPE STREQUAL "Release" OR NOT CMAKE_BUILD_TYPE)
    add_compile_options(-O3)
  endif()
endif()

# Find 3rd party dependencies first for global compile options
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
  add_compile_options(${OpenMP_CXX_FLAGS})
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slam_gnss_2d_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(ublox_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

# Find 3rd party dependencies
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

find_path(NANOFLANN_INCLUDE_DIRS "nanoflann.hpp")

set(DEPENDENCIES
  rclcpp
  slam_gnss_2d_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs
  ublox_msgs
  rosbag2_cpp
  rosbag2_storage
)

# Core library
add_library(slam_gnss_2d_core
  src/core/geometry.cpp
  src/core/config_loader.cpp
  src/core/graph_orchestrator.cpp
  src/core/sensor_synchronizer.cpp
  src/core/slam_data_saver.cpp
  src/core/slam_node_base.cpp
  src/gnss/utm.cpp
  src/gnss/anchor_manager.cpp
  src/input/time_series.cpp
  src/input/ros2/ros_adapter.cpp
  src/input/ros2/bag_reader.cpp
  src/map_manager/grid_utils.cpp
  src/map_manager/overwrite_renderer.cpp
  src/map_manager/counting_renderer.cpp
  src/map_manager/trajectory_noise_filter.cpp
  src/optimizer/isam2_optimizer.cpp
  src/optimizer/gtsam_optimizer.cpp
  src/pose_graph/odom_builder.cpp
  src/pose_graph/scan_matching_builder.cpp
  src/pose_graph/loop_closure_builder.cpp
  src/scan_matching/icp_matcher.cpp
  src/scan_matching/ndt_matcher.cpp
  src/scan_matching/csm_matcher.cpp
  src/scan_matching/multi_res_csm_matcher.cpp
  src/scan_matching/reference_provider/scan_to_scan.cpp
  src/scan_matching/reference_provider/local_map.cpp
  src/ros/slam_visualizer.cpp
  src/ros/tf_broadcaster.cpp
  src/ros/map_save_service.cpp
  src/ros/pose_graph_service.cpp
  src/ros/pose_graph_message_builder.cpp
  src/tools/reoptimize_geometry.cpp
  src/tools/reoptimize_io.cpp
  src/tools/reoptimize_loop_search.cpp
)

target_include_directories(slam_gnss_2d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${NANOFLANN_INCLUDE_DIRS}
)

ament_target_dependencies(slam_gnss_2d_core ${DEPENDENCIES})

target_compile_options(slam_gnss_2d_core PUBLIC ${OpenMP_CXX_FLAGS})

target_link_libraries(slam_gnss_2d_core
  gtsam
  ${OpenCV_LIBS}
  ${GeographicLib_LIBRARIES}
  nlohmann_json::nlohmann_json
  yaml-cpp
  OpenMP::OpenMP_CXX
)

# Executables
add_executable(slam_node src/nodes/slam_node.cpp)
target_link_libraries(slam_node slam_gnss_2d_core)
ament_target_dependencies(slam_node ${DEPENDENCIES})

add_executable(slam_offline_node src/nodes/slam_offline_node.cpp)
target_link_libraries(slam_offline_node slam_gnss_2d_core)
ament_target_dependencies(slam_offline_node ${DEPENDENCIES})

add_executable(reoptimize_node src/nodes/reoptimize_node.cpp)
target_link_libraries(reoptimize_node slam_gnss_2d_core)
ament_target_dependencies(reoptimize_node ${DEPENDENCIES})

add_executable(slam_gnss_nav_bridge_node src/nodes/slam_gnss_nav_bridge_node.cpp)
target_link_libraries(slam_gnss_nav_bridge_node slam_gnss_2d_core)
ament_target_dependencies(slam_gnss_nav_bridge_node ${DEPENDENCIES})

add_executable(anchor_publisher_node src/nodes/anchor_publisher_node.cpp)
target_link_libraries(anchor_publisher_node slam_gnss_2d_core)
ament_target_dependencies(anchor_publisher_node ${DEPENDENCIES})

add_executable(pose_graph_preview_node src/nodes/pose_graph_preview_node.cpp)
target_link_libraries(pose_graph_preview_node slam_gnss_2d_core)
ament_target_dependencies(pose_graph_preview_node ${DEPENDENCIES})

add_executable(save_slam_map_cli src/tools/save_slam_map_cli.cpp)
target_link_libraries(save_slam_map_cli slam_gnss_2d_core)
ament_target_dependencies(save_slam_map_cli ${DEPENDENCIES})

# Install
install(TARGETS
  slam_gnss_2d_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  slam_node
  slam_offline_node
  reoptimize_node
  slam_gnss_nav_bridge_node
  anchor_publisher_node
  pose_graph_preview_node
  save_slam_map_cli
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry slam_gnss_2d_core)

  ament_add_gtest(test_slam_data_saver test/test_slam_data_saver.cpp)
  target_link_libraries(test_slam_data_saver slam_gnss_2d_core)

  ament_add_gtest(test_time_series test/test_time_series.cpp)
  target_link_libraries(test_time_series slam_gnss_2d_core)

  ament_add_gtest(test_bag_time_utils test/test_bag_time_utils.cpp)
  target_link_libraries(test_bag_time_utils slam_gnss_2d_core)

  ament_add_gtest(test_anchor_manager test/test_anchor_manager.cpp)
  target_link_libraries(test_anchor_manager slam_gnss_2d_core)

  ament_add_gtest(test_icp_matcher test/test_icp_matcher.cpp)
  target_link_libraries(test_icp_matcher slam_gnss_2d_core)

  ament_add_gtest(test_ndt_matcher test/test_ndt_matcher.cpp)
  target_link_libraries(test_ndt_matcher slam_gnss_2d_core)

  ament_add_gtest(test_loop_closure test/test_loop_closure.cpp)
  target_link_libraries(test_loop_closure slam_gnss_2d_core)

  ament_add_gtest(test_isam2_optimizer test/test_isam2_optimizer.cpp)
  target_link_libraries(test_isam2_optimizer slam_gnss_2d_core)

  ament_add_gtest(test_counting_renderer test/test_counting_renderer.cpp)
  target_link_libraries(test_counting_renderer slam_gnss_2d_core)

  ament_add_gtest(test_coarse_to_fine_matcher test/test_coarse_to_fine_matcher.cpp)
  target_link_libraries(test_coarse_to_fine_matcher slam_gnss_2d_core)

  ament_add_gtest(test_multi_start_icp_matcher test/test_multi_start_icp_matcher.cpp)
  target_link_libraries(test_multi_start_icp_matcher slam_gnss_2d_core)

  ament_add_gtest(test_multi_start_coarse_to_fine_matcher test/test_multi_start_coarse_to_fine_matcher.cpp)
  target_link_libraries(test_multi_start_coarse_to_fine_matcher slam_gnss_2d_core)

  ament_add_gtest(test_multi_res_csm_matcher test/test_multi_res_csm_matcher.cpp)
  target_link_libraries(test_multi_res_csm_matcher slam_gnss_2d_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(slam_gnss_2d_core)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
