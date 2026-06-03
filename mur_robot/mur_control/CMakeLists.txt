cmake_minimum_required(VERSION 3.8)
project(mur_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(jparse_velocity_controller src/jparse_velocity_controller.cpp)
ament_target_dependencies(jparse_velocity_controller
  geometry_msgs
  kdl_parser
  orocos_kdl
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(jparse_velocity_controller
  Eigen3::Eigen
)

add_executable(arm_velocity_safety_node src/arm_velocity_safety_node.cpp)
ament_target_dependencies(arm_velocity_safety_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/JparseMove.action"
  DEPENDENCIES geometry_msgs
)

install(TARGETS
  arm_velocity_safety_node
  jparse_velocity_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/jparse_move_action_server.py
  scripts/jparse_simple_goal.py
  scripts/moveit_trajectory_controller_proxy.py
  scripts/test_forward_velocity_controller.py
  scripts/test_jparse_velocity_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  action
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
