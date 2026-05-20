"""Launch several MUR bases in one Gazebo world.

Edit ROBOT_POSES below to choose names and spawn poses. Robots are included
one after another because Gazebo is much more stable when model creation is
staggered.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# name, x, y, z, yaw
ROBOT_POSES = [
    ('mur620a', 44.0, 44.0, 0.07, 0.0),
    ('mur620b', 46.0, 44.0, 0.07, 0.0),
    ('mur620c', 48.0, 44.0, 0.07, 0.0),
    ('mur620d', 50.0, 44.0, 0.07, 0.0),
]


def declare_args():
    mir_gazebo_path = get_package_share_directory('mir_gazebo')
    return [
        DeclareLaunchArgument('world', default_value='scale'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(mir_gazebo_path, 'maps', 'scale.yaml'),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'spawn_interval',
            default_value='8.0',
            description='Seconds between two robot includes/spawn attempts.',
        ),
        DeclareLaunchArgument(
            'map_server',
            default_value='true',
            description='Start one shared /map server for all robots.',
        ),
        DeclareLaunchArgument('lidar_bridge', default_value='true'),
        DeclareLaunchArgument('load_controllers', default_value='true'),
        DeclareLaunchArgument('laser_merger', default_value='true'),
        DeclareLaunchArgument('ground_truth', default_value='true'),
        DeclareLaunchArgument('use_simple_collisions', default_value='true'),
        DeclareLaunchArgument(
            'use_simple_visuals',
            default_value='false',
            description='Replace heavy MiR visual meshes with primitives for RViz/GPU diagnosis.',
        ),
        DeclareLaunchArgument(
            'use_high_quality_visuals',
            default_value='false',
            description='Use full-resolution MiR base/top visual meshes instead of simplified meshes.',
        ),
        DeclareLaunchArgument('use_base_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_top_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_wheel_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_caster_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_lift_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_laser_visual_mesh', default_value='false'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Start a lightweight multi-base RViz config.',
        ),
        DeclareLaunchArgument('rviz_delay', default_value='6.0'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('mur_launch_sim'),
                'rviz',
                'multi_mur_base.rviz',
            ),
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='true',
            description='Start one AMCL instance per robot.',
        ),
        DeclareLaunchArgument('navigation', default_value='false'),
    ]


def amcl_params(robot_name, use_sim_time, x, y, yaw):
    return {
        'use_sim_time': use_sim_time,
        'alpha1': 0.35,
        'alpha2': 0.15,
        'alpha3': 0.12,
        'alpha4': 0.35,
        'alpha5': 0.1,
        'base_frame_id': f'{robot_name}/base_footprint',
        'global_frame_id': 'map',
        'map_topic': '/map',
        'map_subscribe_transient_local': True,
        'odom_frame_id': f'{robot_name}/odom',
        'scan_topic': f'/{robot_name}/scan',
        'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
        'laser_model_type': 'likelihood_field',
        'laser_likelihood_max_dist': 2.0,
        'max_beams': 60,
        'max_particles': 1500,
        'min_particles': 200,
        'pf_err': 0.05,
        'pf_z': 0.99,
        'resample_interval': 1,
        'set_initial_pose': True,
        'initial_pose': {
            'x': float(x),
            'y': float(y),
            'z': 0.0,
            'yaw': float(yaw),
        },
        'tf_broadcast': True,
        'transform_tolerance': 0.3,
        'update_min_a': 0.05,
        'update_min_d': 0.05,
    }


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    start_map_server = (
        LaunchConfiguration('map_server').perform(context).lower() == 'true'
    )
    start_localization = (
        LaunchConfiguration('localization').perform(context).lower() == 'true'
    )
    spawn_interval = float(LaunchConfiguration('spawn_interval').perform(context))
    mur_launch_sim_path = get_package_share_directory('mur_launch_sim')
    mir_gazebo_path = get_package_share_directory('mir_gazebo')
    mur_base_launch = os.path.join(mur_launch_sim_path, 'launch', 'mur_base.launch.py')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    )

    actions = [
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.pathsep.join([
                os.path.join(mir_gazebo_path, 'worlds'),
                os.path.join(mir_gazebo_path, 'worlds', 'include'),
                os.path.join(mir_gazebo_path, 'models'),
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': f'{world}.world -v 4 -r'}.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),
    ]

    if start_map_server:
        actions.extend([
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'yaml_filename': map_yaml,
                    'frame_id': 'map',
                    'topic_name': 'map',
                }],
                output='screen',
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server'],
                }],
                output='screen',
            ),
        ])

    for index, (robot_name, x, y, z, yaw) in enumerate(ROBOT_POSES):
        robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mur_base_launch),
            launch_arguments={
                'robot_name': robot_name,
                'world': world,
                'map': LaunchConfiguration('map'),
                'x': str(x),
                'y': str(y),
                'z': str(z),
                'Y': str(yaw),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'include_gz': 'false',
                'lidar_bridge': LaunchConfiguration('lidar_bridge'),
                'load_controllers': LaunchConfiguration('load_controllers'),
                'load_lift_controllers': 'false',
                'laser_merger': LaunchConfiguration('laser_merger'),
                'ground_truth': LaunchConfiguration('ground_truth'),
                'use_arms': 'false',
                'use_camera': 'false',
                'use_simple_collisions': LaunchConfiguration('use_simple_collisions'),
                'use_simple_visuals': LaunchConfiguration('use_simple_visuals'),
                'use_high_quality_visuals': LaunchConfiguration(
                    'use_high_quality_visuals'
                ),
                'use_base_visual_mesh': LaunchConfiguration('use_base_visual_mesh'),
                'use_top_visual_mesh': LaunchConfiguration('use_top_visual_mesh'),
                'use_wheel_visual_mesh': LaunchConfiguration('use_wheel_visual_mesh'),
                'use_caster_visual_mesh': LaunchConfiguration('use_caster_visual_mesh'),
                'use_lift_visual_mesh': LaunchConfiguration('use_lift_visual_mesh'),
                'use_laser_visual_mesh': LaunchConfiguration('use_laser_visual_mesh'),
                'localization': 'false',
                'navigation': 'false',
            }.items(),
        )
        actions.append(TimerAction(period=index * spawn_interval, actions=[robot]))
        if start_localization:
            actions.append(
                TimerAction(
                    period=index * spawn_interval + 3.0,
                    actions=[
                        Node(
                            package='nav2_amcl',
                            executable='amcl',
                            namespace=robot_name,
                            name='amcl',
                            parameters=[amcl_params(robot_name, use_sim_time, x, y, yaw)],
                            output='screen',
                        ),
                        Node(
                            package='nav2_lifecycle_manager',
                            executable='lifecycle_manager',
                            namespace=robot_name,
                            name='lifecycle_manager_localization',
                            parameters=[{
                                'use_sim_time': use_sim_time,
                                'autostart': True,
                                'node_names': ['amcl'],
                            }],
                            output='screen',
                        ),
                    ],
                )
            )

    actions.append(
        TimerAction(
            period=float(LaunchConfiguration('rviz_delay').perform(context)),
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2_multi_mur_base',
                    arguments=['-d', LaunchConfiguration('rviz_config')],
                    output='screen',
                ),
            ],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(declare_args() + [OpaqueFunction(function=launch_setup)])
