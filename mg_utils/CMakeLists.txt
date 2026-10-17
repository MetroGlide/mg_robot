################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(mg_utils)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

################################################################################
# Install
################################################################################
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# ヘッダだけのライブラリ (event_format.hpp、event_log.hpp)
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

################################################################################
# for python nodes
################################################################################
ament_python_install_package(${PROJECT_NAME})

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_event_format test/test_event_format.cpp)
  target_include_directories(test_event_format PRIVATE include)
endif()

################################################################################
# Macro for ament package
################################################################################
ament_package()
