import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    match_gazebo_path = get_package_share_directory('match_gazebo')
    worlds_path = os.path.join(match_gazebo_path, 'worlds')

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([worlds_path, os.path.join(worlds_path, 'include')]),
    )

    arguments = LaunchDescription([
        DeclareLaunchArgument('world', default_value='maze', description='Gz sim World'),
    ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py',
        ]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), '.world', ' -v 4', ' -r'])
        ],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/b_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/f_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo_resource_path,
        arguments,
        gazebo,
        ros_gz_bridge,
    ])
