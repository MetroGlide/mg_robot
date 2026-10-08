################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(mg_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/waypoint_navigator.cpp"
)

set(DEPENDENCIES
  "rclcpp"
  "rcutils"

  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"

  "rclcpp_action"
  "rclcpp_lifecycle"
  "rclcpp_components"

  "nav2_core"
  "nav2_common"
  "nav2_msgs"
  "nav2_util"

  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(WAYPOINT_NAV_EXECUTABLE_NAME "waypoint_navigator_node")
add_executable(${WAYPOINT_NAV_EXECUTABLE_NAME} src/waypoint_navigator.cpp)
target_link_libraries(${WAYPOINT_NAV_EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${WAYPOINT_NAV_EXECUTABLE_NAME} ${DEPENDENCIES})

set(COSTMAP_FOR_BAG_PLAY "costmap_for_bag_play")
add_executable(${COSTMAP_FOR_BAG_PLAY} src/costmap_for_bag_play.cpp)
target_link_libraries(${COSTMAP_FOR_BAG_PLAY} ${PROJECT_NAME}_lib)
ament_target_dependencies(${COSTMAP_FOR_BAG_PLAY} ${DEPENDENCIES})

# bt_navigator が読み込む BT プラグイン (nav2_params.yaml の plugin_lib_names)
set(BT_PLUGIN_DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "nav2_msgs"
  "nav2_util"
  "nav2_costmap_2d"
  "nav2_behavior_tree"
  "behaviortree_cpp_v3"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

add_library(mg_bt_path_clearance SHARED src/bt_plugins/path_clearance.cpp)
ament_target_dependencies(mg_bt_path_clearance ${BT_PLUGIN_DEPENDENCIES})

set(BT_PLUGIN_LIBS
  mg_is_path_clear_condition_bt_node
  mg_commit_path_action_bt_node
)
add_library(mg_is_path_clear_condition_bt_node SHARED
  src/bt_plugins/is_path_clear_condition.cpp)
add_library(mg_commit_path_action_bt_node SHARED
  src/bt_plugins/commit_path_action.cpp)
foreach(bt_plugin ${BT_PLUGIN_LIBS})
  target_link_libraries(${bt_plugin} mg_bt_path_clearance)
  ament_target_dependencies(${bt_plugin} ${BT_PLUGIN_DEPENDENCIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

################################################################################
# Install
################################################################################
install(DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS mg_bt_path_clearance ${BT_PLUGIN_LIBS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${WAYPOINT_NAV_EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${COSTMAP_FOR_BAG_PLAY}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# for python nodes
################################################################################
ament_python_install_package(${PROJECT_NAME})



install(PROGRAMS
  scripts/gnss_amcl_initializer/gnss_amcl_initializer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/amcl_watchdog/amcl_watchdog_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/amcl_gate_arbiter/amcl_gate_arbiter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/localization_supervisor/localization_supervisor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Test (gtest。Python のテストは make test で pytest から実行する)
################################################################################
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_path_clearance test/test_path_clearance.cpp)
  target_link_libraries(test_path_clearance mg_bt_path_clearance)
  ament_target_dependencies(test_path_clearance ${BT_PLUGIN_DEPENDENCIES})

  ament_add_gtest(test_commit_path_action test/test_commit_path_action.cpp)
  target_link_libraries(test_commit_path_action mg_commit_path_action_bt_node)
  ament_target_dependencies(test_commit_path_action ${BT_PLUGIN_DEPENDENCIES})
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()
