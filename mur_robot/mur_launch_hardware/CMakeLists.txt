cmake_minimum_required(VERSION 3.8)
project(mur_launch_hardware)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/ewellix_state_to_joint_state.py
  scripts/ur_startup_enable.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  calibration
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
