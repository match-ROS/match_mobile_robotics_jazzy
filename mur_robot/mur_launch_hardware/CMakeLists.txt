cmake_minimum_required(VERSION 3.8)
project(mur_launch_hardware)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/bms_can_node.py
  scripts/ewellix_dual_state_to_joint_state.py
  scripts/ewellix_state_to_joint_state.py
  scripts/fake_mir_wheel_joint_states.py
  scripts/open_integrated_collision_rviz.sh
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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(robot_joint_states test/test_robot_joint_states.py
    APPEND_ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs
  )
endif()

ament_package()
