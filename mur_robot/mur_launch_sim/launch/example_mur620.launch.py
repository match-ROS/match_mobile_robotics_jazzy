"""Example: start Gazebo with a world and include single robot launch.

This is a convenience wrapper that starts the Gazebo (gz sim) server/client
with the chosen world and then brings up one MUR620 robot by including the
robot-only `mur620.launch.py` description. Users wanting ONLY the robot setup
for an already-running simulation should include/launch `mur620.launch.py`
directly instead.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('world', default_value='maze'),
    ]

    ros_gz_sim_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    mir_gazebo_path = get_package_share_directory('mir_gazebo')
    mur_launch_sim_path = get_package_share_directory('mur_launch_sim')
    mur_single_launch = os.path.join(mur_launch_sim_path, 'launch', 'mur620.launch.py')

    set_res_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(mir_gazebo_path, 'worlds'),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_gz_sim_launch),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), '.world -v 4 -r']
        }.items(),
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    mur_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mur_single_launch)
    )

    return LaunchDescription(declared_arguments + [set_res_path, gazebo, clock_bridge, mur_robot])
