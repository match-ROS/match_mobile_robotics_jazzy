cmake_minimum_required(VERSION 3.8)
project(mur_launch_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(gz_ground_truth_publisher src/gz_ground_truth_publisher.cpp)
ament_target_dependencies(gz_ground_truth_publisher
  geometry_msgs
  nav_msgs
  rclcpp
)
target_link_libraries(gz_ground_truth_publisher
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  gz_ground_truth_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ground_truth_from_pose_tf.py
  scripts/laserscan_frame_republisher.py
  scripts/moveit_move_to_home.py  # Replace with the path to your Python node
  scripts/publish_moveit_descriptions.py
  scripts/test_forward_velocity_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
