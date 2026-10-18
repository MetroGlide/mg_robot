cmake_minimum_required(VERSION 3.8)
project(localization_supervisor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/localization_supervisor_node.py
  scripts/gate_arbiter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
