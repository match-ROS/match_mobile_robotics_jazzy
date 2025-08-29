"""Multi-robot mur620 launch (robot-only, no Gazebo world).

Spawns four mur620 robots (a-d) into an ALREADY running Gazebo instance. It
does not start a world. For a ready-to-run example including Gazebo, use
`example_multi_mur620.launch.py`.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mur_launch_sim_path = get_package_share_directory('mur_launch_sim')
    mur_base_launch = os.path.join(mur_launch_sim_path, 'launch', 'mur_base.launch.py')

    def robot_inc(name, x):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mur_base_launch),
            launch_arguments={
                'robot_name': name,
                'x': str(x), 'y': '0.0', 'z': '0.07', 'Y': '0.0',
                'include_gz': 'false',  # world handled elsewhere
                'lidar_bridge': 'true',
            }.items()
        )

    robots = [
        robot_inc('mur620a', 0.0),
        robot_inc('mur620b', 1.5),
        robot_inc('mur620c', 3.5),
        robot_inc('mur620d', 5.5),
    ]

    return LaunchDescription(robots)
