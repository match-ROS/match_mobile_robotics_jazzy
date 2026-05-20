"""Compatibility wrapper for the multi-MUR620 launch file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mur_launch_sim_path = get_package_share_directory('mur_launch_sim')
    multi_launch = os.path.join(
        mur_launch_sim_path,
        'launch',
        'multi_mur620.launch.py',
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(multi_launch)),
    ])
