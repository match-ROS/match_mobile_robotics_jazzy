"""Single mur620 robot launch (robot-only, no Gazebo world).

This file ONLY sets up and spawns one MUR620 robot (mur620a) assuming a Gazebo
simulation is already running (started elsewhere, e.g. in an example launch
file). It intentionally does NOT start a world. For a ready-to-run example
that also starts Gazebo, use `example_mur620.launch.py`.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mur_launch_sim_path = get_package_share_directory('mur_launch_sim')
    mur_base_launch = os.path.join(mur_launch_sim_path, 'launch', 'mur_base.launch.py')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mur_base_launch),
        launch_arguments={
            'robot_name': 'mur620a',
            'x': '0.0', 'y': '0.0', 'z': '0.07', 'Y': '0.0',
            'include_gz': 'false',  # world must be started by including launch
            'lidar_bridge': 'true',
        }.items()
    )

    return LaunchDescription([robot])