import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, OpaqueFunction, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command, IfElseSubstitution, EnvironmentVariable, SubstitutionFailure
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

  
def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    match_gazebo_path = os.path.join(get_package_share_directory('match_gazebo'))

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(match_gazebo_path, 'worlds')
            ]
        )

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='maze',
                          description='Gz sim World'),
           ]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.world',
                                 ' -v 4',
                                 ' -r']
                    )
                ]
             )
    
    
    # Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/b_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/f_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'  # <--- Das ist neu!
        ],
        output='screen'
)

    return LaunchDescription([
        gazebo_resource_path,
        arguments,
        gazebo,
        ros_gz_bridge,
    ])


