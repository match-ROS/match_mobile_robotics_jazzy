"""Single MUR620 launch.

This wraps ``mur_base.launch.py`` and adds the arm layer needed for simulated
UR10 control: ros2_control trajectory controllers for both arms and, optionally,
MoveIt. By default it keeps the historical robot-only behavior and assumes a
Gazebo world is already running; pass ``include_gz:=true`` for a standalone
simulation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def controller_spawner(controller_name, *, inactive=False):
    robot_name = LaunchConfiguration('robot_name')
    arguments = [
        controller_name,
        '--controller-manager',
        ['/', robot_name, '/controller_manager'],
        '--controller-manager-timeout',
        '90',
        '--param-file',
        ['/tmp/mur_launch_sim/', robot_name, '_mur_controllers.yaml'],
    ]
    if inactive:
        arguments.append('--inactive')

    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=arguments,
        output='screen',
    )


def generate_launch_description():
    mur_launch_sim_path = get_package_share_directory('mur_launch_sim')
    mur_moveit_path = get_package_share_directory('mur_moveit_config')
    mir_gazebo_path = get_package_share_directory('mir_gazebo')

    mur_base_launch = os.path.join(mur_launch_sim_path, 'launch', 'mur_base.launch.py')
    moveit_launch = os.path.join(mur_moveit_path, 'launch', 'ur_moveit.launch.py')

    declared_arguments = [
        DeclareLaunchArgument('robot_name', default_value='mur620a'),
        DeclareLaunchArgument('world', default_value='scale'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(mir_gazebo_path, 'maps', 'scale.yaml'),
        ),
        DeclareLaunchArgument('x', default_value='44.0'),
        DeclareLaunchArgument('y', default_value='44.0'),
        DeclareLaunchArgument('z', default_value='0.07'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('include_gz', default_value='false'),
        DeclareLaunchArgument('lidar_bridge', default_value='true'),
        DeclareLaunchArgument('start_controller_manager', default_value='false'),
        DeclareLaunchArgument('load_controllers', default_value='true'),
        DeclareLaunchArgument('laser_merger', default_value='true'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('navigation', default_value='false'),
        DeclareLaunchArgument('ground_truth', default_value='true'),
        DeclareLaunchArgument('use_simple_collisions', default_value='false'),
        DeclareLaunchArgument('use_simple_visuals', default_value='false'),
        DeclareLaunchArgument('use_high_quality_visuals', default_value='false'),
        DeclareLaunchArgument('use_base_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_top_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_wheel_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_caster_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_lift_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_laser_visual_mesh', default_value='false'),
        DeclareLaunchArgument('load_arm_controllers', default_value='true'),
        DeclareLaunchArgument('launch_moveit', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_delay', default_value='10.0'),
        DeclareLaunchArgument('launch_servo', default_value='false'),
        DeclareLaunchArgument('auto_switch_arm_controllers', default_value='true'),
        DeclareLaunchArgument('launch_jparse_idk', default_value='true'),
        DeclareLaunchArgument('ur_type', default_value='ur10e'),
    ]

    base_arguments = {
        'robot_name': LaunchConfiguration('robot_name'),
        'world': LaunchConfiguration('world'),
        'x': LaunchConfiguration('x'),
        'y': LaunchConfiguration('y'),
        'z': LaunchConfiguration('z'),
        'Y': LaunchConfiguration('Y'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'include_gz': LaunchConfiguration('include_gz'),
        'lidar_bridge': LaunchConfiguration('lidar_bridge'),
        'start_controller_manager': LaunchConfiguration('start_controller_manager'),
        'load_controllers': LaunchConfiguration('load_controllers'),
        'laser_merger': LaunchConfiguration('laser_merger'),
        'localization': LaunchConfiguration('localization'),
        'navigation': LaunchConfiguration('navigation'),
        'ground_truth': LaunchConfiguration('ground_truth'),
        'use_arms': 'true',
        'use_camera': 'true',
        'use_simple_collisions': LaunchConfiguration('use_simple_collisions'),
        'use_simple_visuals': LaunchConfiguration('use_simple_visuals'),
        'use_high_quality_visuals': LaunchConfiguration('use_high_quality_visuals'),
        'use_base_visual_mesh': LaunchConfiguration('use_base_visual_mesh'),
        'use_top_visual_mesh': LaunchConfiguration('use_top_visual_mesh'),
        'use_wheel_visual_mesh': LaunchConfiguration('use_wheel_visual_mesh'),
        'use_caster_visual_mesh': LaunchConfiguration('use_caster_visual_mesh'),
        'use_lift_visual_mesh': LaunchConfiguration('use_lift_visual_mesh'),
        'use_laser_visual_mesh': LaunchConfiguration('use_laser_visual_mesh'),
    }

    base_arguments['map'] = LaunchConfiguration('map')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mur_base_launch),
        launch_arguments=base_arguments.items(),
    )

    arm_controllers = TimerAction(
        period=3.0,
        condition=IfCondition(LaunchConfiguration('load_arm_controllers')),
        actions=[
            controller_spawner('forward_velocity_controller_l'),
            controller_spawner('forward_velocity_controller_r'),
            controller_spawner('joint_trajectory_controller_l', inactive=True),
            controller_spawner('joint_trajectory_controller_r', inactive=True),
            controller_spawner('joint_trajectory_controller_lift_l', inactive=True),
            controller_spawner('joint_trajectory_controller_lift_r', inactive=True),
        ],
    )

    moveit_descriptions = Node(
        package='mur_launch_sim',
        executable='publish_moveit_descriptions.py',
        name=[LaunchConfiguration('robot_name'), '_moveit_descriptions'],
        condition=IfCondition(LaunchConfiguration('launch_moveit')),
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )

    arm_controller_switchers = TimerAction(
        period=3.0,
        condition=IfCondition(LaunchConfiguration('auto_switch_arm_controllers')),
        actions=[
            Node(
                package='mur_launch_sim',
                executable='moveit_trajectory_controller_proxy.py',
                name=[LaunchConfiguration('robot_name'), '_moveit_controller_proxy_l'],
                arguments=['--robot-name', LaunchConfiguration('robot_name'), '--arm', 'l'],
                output='screen',
            ),
            Node(
                package='mur_launch_sim',
                executable='moveit_trajectory_controller_proxy.py',
                name=[LaunchConfiguration('robot_name'), '_moveit_controller_proxy_r'],
                arguments=['--robot-name', LaunchConfiguration('robot_name'), '--arm', 'r'],
                output='screen',
            ),
            Node(
                package='mur_launch_sim',
                executable='moveit_trajectory_controller_proxy.py',
                name=[LaunchConfiguration('robot_name'), '_moveit_lift_controller_proxy_l'],
                arguments=[
                    '--robot-name', LaunchConfiguration('robot_name'),
                    '--arm', 'l',
                    '--include-lift',
                ],
                output='screen',
            ),
            Node(
                package='mur_launch_sim',
                executable='moveit_trajectory_controller_proxy.py',
                name=[LaunchConfiguration('robot_name'), '_moveit_lift_controller_proxy_r'],
                arguments=[
                    '--robot-name', LaunchConfiguration('robot_name'),
                    '--arm', 'r',
                    '--include-lift',
                ],
                output='screen',
            ),
        ],
    )

    jparse_idk = TimerAction(
        period=4.0,
        condition=IfCondition(LaunchConfiguration('launch_jparse_idk')),
        actions=[
            Node(
                package='mur_launch_sim',
                executable='jparse_velocity_controller',
                name=[LaunchConfiguration('robot_name'), '_jparse_velocity_controller_l'],
                parameters=[{
                    'robot_name': LaunchConfiguration('robot_name'),
                    'arm': 'l',
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
                remappings=[
                    (
                        '~/twist_cmd',
                        ['/', LaunchConfiguration('robot_name'), '/jparse_velocity_controller_l/twist_cmd'],
                    ),
                ],
                output='screen',
            ),
            Node(
                package='mur_launch_sim',
                executable='jparse_velocity_controller',
                name=[LaunchConfiguration('robot_name'), '_jparse_velocity_controller_r'],
                parameters=[{
                    'robot_name': LaunchConfiguration('robot_name'),
                    'arm': 'r',
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
                remappings=[
                    (
                        '~/twist_cmd',
                        ['/', LaunchConfiguration('robot_name'), '/jparse_velocity_controller_r/twist_cmd'],
                    ),
                ],
                output='screen',
            ),
            Node(
                package='mur_launch_sim',
                executable='jparse_move_action_server.py',
                name=[LaunchConfiguration('robot_name'), '_jparse_move_l'],
                arguments=[
                    '--robot-name', LaunchConfiguration('robot_name'),
                    '--arm', 'l',
                ],
                output='screen',
            ),
            Node(
                package='mur_launch_sim',
                executable='jparse_move_action_server.py',
                name=[LaunchConfiguration('robot_name'), '_jparse_move_r'],
                arguments=[
                    '--robot-name', LaunchConfiguration('robot_name'),
                    '--arm', 'r',
                ],
                output='screen',
            ),
            Node(
                package='mur_launch_sim',
                executable='jparse_simple_goal.py',
                name=[LaunchConfiguration('robot_name'), '_jparse_simple_goal_l'],
                arguments=[
                    '--robot-name', LaunchConfiguration('robot_name'),
                    '--arm', 'l',
                ],
                output='screen',
            ),
            Node(
                package='mur_launch_sim',
                executable='jparse_simple_goal.py',
                name=[LaunchConfiguration('robot_name'), '_jparse_simple_goal_r'],
                arguments=[
                    '--robot-name', LaunchConfiguration('robot_name'),
                    '--arm', 'r',
                ],
                output='screen',
            ),
        ],
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        condition=IfCondition(LaunchConfiguration('launch_moveit')),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'launch_servo': LaunchConfiguration('launch_servo'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'rviz_delay': LaunchConfiguration('rviz_delay'),
            'controller_namespace': LaunchConfiguration('robot_name'),
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot,
            arm_controllers,
            moveit_descriptions,
            arm_controller_switchers,
            jparse_idk,
            moveit,
        ]
    )
