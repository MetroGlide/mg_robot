cmake_minimum_required(VERSION 3.8)
project(localization_supervisor_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalizationStatus.msg"
  "msg/LocalizationCheck.msg"
  "msg/GateState.msg"
  "srv/SetProfile.srv"
  DEPENDENCIES std_msgs diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
