"""Single mur620 robot launch that wraps mur_base.launch.py.

Starts Gazebo, spawns one robot (mur620a) with lidar bridge.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('world', default_value='maze'),
    ]

    mur_launch_sim_path = get_package_share_directory('mur_launch_sim')
    mur_base_launch = os.path.join(mur_launch_sim_path, 'launch', 'mur_base.launch.py')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mur_base_launch),
        launch_arguments={
            'robot_name': 'mur620a',
            'x': '0.0', 'y': '0.0', 'z': '0.07', 'Y': '0.0',
            'world': LaunchConfiguration('world'),
            'include_gz': 'true',
            'lidar_bridge': 'true',
        }.items()
    )

    return LaunchDescription(declared_arguments + [robot])