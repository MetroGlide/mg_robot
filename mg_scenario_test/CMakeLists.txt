cmake_minimum_required(VERSION 3.5)
project(mg_scenario_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY profiles scenarios
  DESTINATION share/${PROJECT_NAME}
)

# sim_scenario_test がプロファイル名から share/<pkg>/profiles を探せるよう登録する
ament_index_register_resource("sim_scenario_test.profiles")

ament_package()
