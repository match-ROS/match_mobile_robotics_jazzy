cmake_minimum_required(VERSION 3.8)
project(mur_launch_hardware)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  calibration
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
