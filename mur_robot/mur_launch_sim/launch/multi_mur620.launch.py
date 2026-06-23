"""Launch several full MUR620 mobile manipulators in one Gazebo world.

Edit ROBOT_POSES below to choose names and spawn poses. The launch includes
mur620.launch.py for each robot with a delay between includes so Gazebo does
not have to create every full model at once.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# name, x, y, z, yaw
ROBOT_POSES = [
    ('mur620a', 44.0, 44.0, 0.07, 0.0),
    ('mur620b', 46.0, 44.0, 0.07, 0.0),
]

INTEGRATED_CONTROLLER_ARGUMENT_DEFAULTS = {
    'use_integrated_cartesian_admittance_controller': 'false',
    'integrated_controller_initial_active': 'false',
    'integrated_controller_use_ft_sensor': 'false',
    'integrated_controller_require_wrench': 'false',
    'integrated_controller_enable_collision_avoidance': 'true',
    'integrated_controller_publish_collision_markers': 'false',
    'integrated_controller_admittance': '0.0 0.0 0.0 0.0 0.0 0.0',
    'integrated_controller_wrench_twist_gain': '0.0 0.0 0.0 0.0 0.0 0.0',
}


def integrated_launch_arguments():
    return [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in INTEGRATED_CONTROLLER_ARGUMENT_DEFAULTS.items()
    ]


def integrated_forward_arguments():
    return {
        name: LaunchConfiguration(name)
        for name in INTEGRATED_CONTROLLER_ARGUMENT_DEFAULTS
    }


def parse_robot_selection(value):
    known_robots = [robot[0] for robot in ROBOT_POSES]
    names = [name for name in re.split(r'[\s,;]+', value.strip()) if name]

    if not names:
        return []
    if any(name.lower() == 'all' for name in names):
        return known_robots

    return [name for name in names if name in known_robots]


def select_rviz_robot(value, moveit_robots):
    requested = value.strip()
    if requested and requested in moveit_robots:
        return requested
    return moveit_robots[0] if moveit_robots else None


def rviz_config_for_robot(source_config, robot_name):
    target_config = os.path.join('/tmp', f'multi_mur620_{robot_name}.rviz')
    with open(source_config, 'r', encoding='utf-8') as source:
        contents = source.read()

    default_robot = ROBOT_POSES[0][0]
    contents = contents.replace(
        f'Move Group Namespace: {default_robot}',
        f'Move Group Namespace: {robot_name}',
    )
    for old_topic in (
        'Planning Scene Topic: monitored_planning_scene',
        f'Planning Scene Topic: /{default_robot}/monitored_planning_scene',
    ):
        contents = contents.replace(
            old_topic,
            f'Planning Scene Topic: /{robot_name}/monitored_planning_scene',
        )
    for old_topic in (
        'Trajectory Topic: display_planned_path',
        f'Trajectory Topic: /{default_robot}/display_planned_path',
    ):
        contents = contents.replace(
            old_topic,
            f'Trajectory Topic: /{robot_name}/display_planned_path',
        )
    with open(target_config, 'w', encoding='utf-8') as target:
        target.write(contents)

    return target_config


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
            'gazebo_gui',
            default_value='false',
            description='Start Gazebo GUI in addition to the server.',
        ),
        DeclareLaunchArgument(
            'spawn_interval',
            default_value='12.0',
            description='Seconds between two full robot includes/spawn attempts.',
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
            'localization',
            default_value='true',
            description='Start one AMCL instance per robot.',
        ),
        DeclareLaunchArgument(
            'fake_localization',
            default_value='false',
            description='Use Gazebo ground truth for map->odom instead of AMCL and disable lidar.',
        ),
        DeclareLaunchArgument('navigation', default_value='false'),
        DeclareLaunchArgument('load_arm_controllers', default_value='true'),
        DeclareLaunchArgument('auto_switch_arm_controllers', default_value='true'),
        DeclareLaunchArgument('launch_jparse_idk', default_value='true'),
        DeclareLaunchArgument(
            'launch_moveit',
            default_value='false',
            description='Start MoveIt for the robots selected by moveit_robot.',
        ),
        DeclareLaunchArgument(
            'moveit_robot',
            default_value=ROBOT_POSES[0][0],
            description=(
                'Comma or space separated robot names that should launch MoveIt. '
                'Use "all" for every robot. When launch_rviz is true, RViz is '
                'launched for the first selected robot only.'
            ),
        ),
        DeclareLaunchArgument(
            'isolate_moveit_tf',
            default_value='true',
            description=(
                'Give each MoveIt instance private TF topics so several '
                'unprefixed MoveIt robot models do not share one base_footprint.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz_robot',
            default_value='',
            description=(
                'Robot namespace RViz should be configured for. Empty uses the '
                'first selected MoveIt robot. With isolated TF, one RViz can '
                'only drive the robot it was launched for.'
            ),
        ),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('launch_servo', default_value='false'),
        DeclareLaunchArgument('rviz_delay', default_value='14.0'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('mur_launch_sim'),
                'rviz',
                'multi_mur620.rviz',
            ),
        ),
        DeclareLaunchArgument('ur_type', default_value='ur10e'),
    ] + integrated_launch_arguments()


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
    gazebo_gui = LaunchConfiguration('gazebo_gui').perform(context).lower() == 'true'
    start_map_server = (
        LaunchConfiguration('map_server').perform(context).lower() == 'true'
    )
    start_localization = (
        LaunchConfiguration('localization').perform(context).lower() == 'true'
    )
    start_fake_localization = (
        LaunchConfiguration('fake_localization').perform(context).lower() == 'true'
    )
    start_amcl = start_localization and not start_fake_localization
    start_moveit = LaunchConfiguration('launch_moveit').perform(context).lower() == 'true'
    start_rviz = LaunchConfiguration('launch_rviz').perform(context).lower() == 'true'
    isolate_moveit_tf = (
        LaunchConfiguration('isolate_moveit_tf').perform(context).lower() == 'true'
    )
    moveit_robots = parse_robot_selection(LaunchConfiguration('moveit_robot').perform(context))
    rviz_robot = select_rviz_robot(
        LaunchConfiguration('rviz_robot').perform(context), moveit_robots
    )
    spawn_interval = float(LaunchConfiguration('spawn_interval').perform(context))
    mur_launch_sim_path = get_package_share_directory('mur_launch_sim')
    mir_gazebo_path = get_package_share_directory('mir_gazebo')
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    mur620_launch = os.path.join(mur_launch_sim_path, 'launch', 'mur620.launch.py')
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
            launch_arguments={
                'gz_args': (
                    f'{world}.world -v 4 -r'
                    if gazebo_gui
                    else f'-s {world}.world -v 4 -r'
                )
            }.items(),
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
        robot_uses_moveit = start_moveit and robot_name in moveit_robots
        robot_is_rviz_robot = robot_name == rviz_robot
        robot_launches_rviz = (
            start_rviz and robot_uses_moveit and robot_is_rviz_robot
        )
        moveit_tf_topic = f'/{robot_name}/moveit_tf' if isolate_moveit_tf else '/tf'
        moveit_tf_static_topic = (
            f'/{robot_name}/moveit_tf_static' if isolate_moveit_tf else '/tf_static'
        )
        robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mur620_launch),
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
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'lidar_bridge': 'false' if start_fake_localization else LaunchConfiguration('lidar_bridge'),
                'load_controllers': LaunchConfiguration('load_controllers'),
                'laser_merger': 'false' if start_fake_localization else LaunchConfiguration('laser_merger'),
                'ground_truth': 'true' if start_fake_localization else LaunchConfiguration('ground_truth'),
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
                'fake_localization': 'true' if start_fake_localization else 'false',
                'navigation': 'false',
                'load_arm_controllers': LaunchConfiguration('load_arm_controllers'),
                'auto_switch_arm_controllers': LaunchConfiguration(
                    'auto_switch_arm_controllers'
                ),
                'launch_jparse_idk': LaunchConfiguration('launch_jparse_idk'),
                'publish_global_moveit_descriptions': (
                    'true' if robot_is_rviz_robot else 'false'
                ),
                'launch_moveit': 'true' if robot_uses_moveit else 'false',
                'launch_rviz': 'true' if robot_launches_rviz else 'false',
                'rviz_config': rviz_config_for_robot(rviz_config, robot_name),
                'publish_tf_alias': (
                    'false' if (robot_uses_moveit and isolate_moveit_tf)
                    else 'true' if robot_is_rviz_robot
                    else 'false'
                ),
                'tf_topic': moveit_tf_topic,
                'tf_static_topic': moveit_tf_static_topic,
                'virtual_joint_parent_frame': (
                    f'{robot_name}/base_footprint' if robot_uses_moveit else ''
                ),
                'launch_servo': LaunchConfiguration('launch_servo'),
                'rviz_delay': LaunchConfiguration('rviz_delay'),
                'ur_type': LaunchConfiguration('ur_type'),
                **integrated_forward_arguments(),
            }.items(),
        )
        actions.append(TimerAction(period=index * spawn_interval, actions=[robot]))
        if robot_uses_moveit and isolate_moveit_tf:
            actions.append(
                TimerAction(
                    period=index * spawn_interval + 1.0,
                    actions=[
                        Node(
                            package='mur_launch_sim',
                            executable='moveit_tf_republisher.py',
                            name=f'{robot_name}_moveit_tf_republisher',
                            arguments=[
                                '--robot-name', robot_name,
                                '--tf-topic', moveit_tf_topic,
                                '--tf-static-topic', moveit_tf_static_topic,
                            ],
                            output='screen',
                        )
                    ],
                )
            )
        if start_amcl:
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

    if start_rviz and not start_moveit:
        actions.append(
            TimerAction(
                period=float(LaunchConfiguration('rviz_delay').perform(context)),
                actions=[
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        name='rviz2_multi_mur620',
                        arguments=['-d', LaunchConfiguration('rviz_config')],
                        output='screen',
                    ),
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(declare_args() + [OpaqueFunction(function=launch_setup)])
