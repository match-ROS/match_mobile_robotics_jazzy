"""Generic MUR base launch file.

This file is intentionally lightweight and parameterized so it can be included
by single-robot (mur620.launch.py) and multi-robot (multi_mur620.launch.py) launch
descriptions. It can also be launched directly for a quick single robot test.

Responsibilities (per include):
  * Optionally start Gazebo (gz sim) with a selected world (first include only)
  * Process mur_620 xacro and publish robot_description (scoped + global)
  * Spawn the entity at a given pose
  * Start ros2_control node (namespaced) and (optionally) lidar bridge
  * Bridge /clock only once (first include)

Arguments:
  robot_name (string)      Name / namespace / tf prefix (default 'mur620a')
  world (string)           World (without .world) for first include (default 'maze')
  x,y,z,Y (float)          Spawn pose (z default 0.07)
  use_sim_time (bool)      Use simulation time (default true)
  include_gz (bool)        Whether to start gz sim and /clock bridge (default true)
  lidar_bridge (bool)      Whether to bridge the robot's /scan topic (default true)
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_args():
    return [
        DeclareLaunchArgument('robot_name', default_value='mur620a'),
        DeclareLaunchArgument('world', default_value='maze'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.07'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('include_gz', default_value='true'),
        DeclareLaunchArgument('lidar_bridge', default_value='true'),
    ]


def launch_setup(context, *args, **kwargs):  # executed at runtime
    robot_name = LaunchConfiguration('robot_name').perform(context)
    world = LaunchConfiguration('world').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    Y = LaunchConfiguration('Y').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    include_gz = LaunchConfiguration('include_gz').perform(context) == 'true'
    lidar_bridge = LaunchConfiguration('lidar_bridge').perform(context) == 'true'

    mur_description_path = get_package_share_directory('mur_description')
    xacro_file = os.path.join(mur_description_path, 'urdf', 'mur_620.gazebo.xacro')
    controllers_yaml = os.path.join(mur_description_path, 'config', 'mur_controllers.yaml')
    doc = xacro.process_file(xacro_file, mappings={
        'use_sim': 'true',
        'tf_prefix': robot_name,
        'robot_namespace': robot_name,
    })
    robot_desc = doc.toxml()

    nodes = []

    if include_gz:
        mir_gazebo_path = get_package_share_directory('mir_gazebo')
        nodes.append(
            SetEnvironmentVariable(
                name='GZ_SIM_RESOURCE_PATH',
                value=os.path.join(mir_gazebo_path, 'worlds'),
            )
        )
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': f'{world}.world -v 4 -r'}.items(),
            )
        )
        # clock bridge only once
        nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='clock_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
            )
        )

    # state publisher (namespaced)
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_rsp',
            namespace=robot_name,
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': use_sim_time},
                {'frame_prefix': f'{robot_name}/'},
            ],
            output='screen',
        )
    )

    # global (for gz_ros2_control consumption if required)
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_global_rsp',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': use_sim_time},
                {'publish_frequency': 0.0},  # suppress periodic republishes (supported in newer versions)
            ],
            remappings=[('/tf', 'unused_tf'), ('/tf_static', 'unused_tf_static')],  # optional to avoid duplicates
            output='screen',
        )
    )

    # controller manager (namespaced)
    nodes.append(
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=robot_name,
            name='controller_manager',
            parameters=[
                controllers_yaml,  # loads controller definitions
                {'robot_description': robot_desc},  # ensure param present early
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        )
    )

    # spawn entity
    nodes.append(
        Node(
            package='ros_gz_sim',
            executable='create',
            name=f'{robot_name}_spawn',
            arguments=[
                '-string', robot_desc,
                '-name', robot_name,
                '-x', x, '-y', y, '-z', z,
                '-Y', Y,
            ],
            output='screen',
        )
    )

    if lidar_bridge:
        nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{robot_name}_lidar_bridge',
                arguments=[f'/{robot_name}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    ld = LaunchDescription(declare_args())
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld


