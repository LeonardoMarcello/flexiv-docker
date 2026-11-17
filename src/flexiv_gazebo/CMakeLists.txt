cmake_minimum_required(VERSION 3.8)
project(flexiv_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/plot_model_check.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
