cmake_minimum_required(VERSION 3.8)
project(mur_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
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

add_executable(cartesian_admittance_controller src/cartesian_admittance_controller.cpp)
ament_target_dependencies(cartesian_admittance_controller
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

add_library(integrated_cartesian_admittance_controller SHARED
  src/integrated_cartesian_admittance_controller.cpp
)
target_include_directories(integrated_cartesian_admittance_controller
  PRIVATE include
)
ament_target_dependencies(integrated_cartesian_admittance_controller
  controller_interface
  geometry_msgs
  hardware_interface
  kdl_parser
  orocos_kdl
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)
target_link_libraries(integrated_cartesian_admittance_controller
  Eigen3::Eigen
)
pluginlib_export_plugin_description_file(
  controller_interface mur_control_plugins.xml
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/JparseMove.action"
  DEPENDENCIES geometry_msgs
)

install(TARGETS
  arm_velocity_safety_node
  cartesian_admittance_controller
  integrated_cartesian_admittance_controller
  jparse_velocity_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  mur_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/diagnose_cartesian_admittance.py
  scripts/diagnose_integrated_cartesian_admittance_controller.py
  scripts/integrated_cartesian_move_action_server.py
  scripts/integrated_cartesian_jog_gui.py
  scripts/jparse_move_action_server.py
  scripts/jparse_simple_goal.py
  scripts/log_integrated_cartesian_step_response.py
  scripts/moveit_trajectory_controller_proxy.py
  scripts/log_jparse_axis_response.py
  scripts/test_cartesian_admittance_controller.py
  scripts/test_forward_velocity_controller.py
  scripts/test_jparse_velocity_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  action
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_collision_response
    test/test_collision_response.cpp
  )
  target_include_directories(test_collision_response
    PRIVATE include
  )

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
