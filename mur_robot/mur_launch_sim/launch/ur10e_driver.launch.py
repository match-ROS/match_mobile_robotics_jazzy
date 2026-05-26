from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    robot_description_topic = LaunchConfiguration("robot_description_topic")

    ur_driver = GroupAction(
        actions=[
            SetRemap(src="/robot_description", dst=robot_description_topic),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ur_robot_driver"),
                            "launch",
                            "ur_control.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "reverse_ip": reverse_ip,
                    "launch_rviz": launch_rviz,
                    "initial_joint_controller": initial_joint_controller,
                    "activate_joint_controller": activate_joint_controller,
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur10e",
                choices=[
                    "ur3",
                    "ur3e",
                    "ur5",
                    "ur5e",
                    "ur10",
                    "ur10e",
                    "ur16e",
                    "ur20",
                    "ur30",
                ],
                description="UR robot type.",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                description="IP address of the UR controller.",
            ),
            DeclareLaunchArgument(
                "reverse_ip",
                default_value="0.0.0.0",
                description=(
                    "IP address of this ROS PC as reachable from the robot. "
                    "Use 0.0.0.0 to let ur_robot_driver choose it from routing."
                ),
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Launch the UR driver's RViz instance.",
            ),
            DeclareLaunchArgument(
                "initial_joint_controller",
                default_value="scaled_joint_trajectory_controller",
                description="Controller to start for trajectory execution.",
            ),
            DeclareLaunchArgument(
                "activate_joint_controller",
                default_value="true",
                description="Activate the initial joint controller on startup.",
            ),
            DeclareLaunchArgument(
                "robot_description_topic",
                default_value="/ur/robot_description",
                description=(
                    "Robot description topic used by the UR driver. Keeping this "
                    "separate avoids collisions with other robot_state_publishers."
                ),
            ),
            ur_driver,
        ]
    )
