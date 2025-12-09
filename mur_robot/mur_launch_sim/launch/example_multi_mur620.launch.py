"""Example: start Gazebo with a world and include multi-robot launch.

Wrapper that starts Gazebo and then includes `multi_mur620.launch.py` which
spawns four robots. Use `multi_mur620.launch.py` directly when Gazebo is
already running and you just want to add robots.
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
    multi_launch = os.path.join(mur_launch_sim_path, 'launch', 'multi_mur620.launch.py')

    set_res_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(mir_gazebo_path, 'worlds'),
    )

    # Simpler: construct gz_args using substitution formatting
    # We'll just use an OpaqueFunction in future if needed; for now mirror existing pattern in mur_base
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_gz_sim_launch),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), '.world -v 4 -r']
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(multi_launch)
    )

    return LaunchDescription(declared_arguments + [set_res_path, gazebo, clock_bridge, robots])
