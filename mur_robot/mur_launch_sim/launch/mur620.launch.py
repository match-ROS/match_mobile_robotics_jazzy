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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def controller_spawner(controller_name, *, inactive=False, controller_ros_args=None):
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
    if controller_ros_args:
        arguments.extend(['--controller-ros-args', controller_ros_args])

    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=arguments,
        output='screen',
    )


INTEGRATED_CONTROLLER_ARGUMENT_DEFAULTS = {
    'ur_l_xyz': '0.0 0.0 0.0',
    'ur_l_rpy': '0.0 0.0 0.0',
    'ur_r_xyz': '0.0 0.0 0.0',
    'ur_r_rpy': '0.0 0.0 3.14159265359',
    'use_integrated_cartesian_admittance_controller': 'false',
    'integrated_controller_initial_active': 'false',
    'integrated_controller_use_ft_sensor': 'false',
    'integrated_controller_require_wrench': 'false',
    'integrated_controller_command_timeout': '0.12',
    'integrated_controller_wrench_timeout': '0.5',
    'integrated_controller_wrench_bias_duration': '1.0',
    'integrated_controller_wrench_filter_alpha': '0.20',
    'integrated_controller_wrench_in_tcp_frame': 'true',
    'integrated_controller_wrench_sign': '1.0 1.0 1.0 1.0 1.0 1.0',
    'integrated_controller_force_deadband': '0.2',
    'integrated_controller_torque_deadband': '0.05',
    'integrated_controller_admittance': '0.0 0.0 0.0 0.0 0.0 0.0',
    'integrated_controller_wrench_twist_gain': '0.0 0.0 0.0 0.0 0.0 0.0',
    'integrated_controller_pose_error_gain': '1.9 1.9 1.7 0.75 0.75 0.75',
    'integrated_controller_max_linear_velocity': '0.16',
    'integrated_controller_max_angular_velocity': '0.55',
    'integrated_controller_max_joint_velocity': '0.6',
    'integrated_controller_max_joint_acceleration': '1.4',
    'integrated_controller_max_joint_jerk': '6.0',
    'integrated_controller_joint_limit_margin': '0.02',
    'integrated_controller_preserve_command_direction': 'true',
    'integrated_controller_immediate_zero_on_zero_command': 'true',
    'integrated_controller_zero_command_deadband': '1.0e-5',
    'integrated_controller_reset_equilibrium_on_zero_command': 'auto',
    'integrated_controller_enable_collision_avoidance': 'true',
    'integrated_controller_collision_common_link': 'base_link',
    'integrated_controller_collision_joint_states_topic': '/joint_states',
    'integrated_controller_collision_joint_state_timeout': '0.1',
    'integrated_controller_collision_sample_spacing': '0.08',
    'integrated_controller_collision_sphere_radius': '0.04',
    'integrated_controller_collision_activation_clearance': '0.08',
    'integrated_controller_collision_stop_clearance': '0.035',
    'integrated_controller_collision_fail_safe_stop': 'true',
    'integrated_controller_collision_forbidden_boxes': 'default',
    'integrated_controller_publish_collision_markers': 'false',
    'integrated_controller_collision_marker_publish_rate_hz': '10.0',
}


def integrated_launch_arguments():
    return [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in INTEGRATED_CONTROLLER_ARGUMENT_DEFAULTS.items()
    ]


def integrated_base_arguments():
    return {
        name: LaunchConfiguration(name)
        for name in INTEGRATED_CONTROLLER_ARGUMENT_DEFAULTS
        if name != 'integrated_controller_initial_active'
    }


def integrated_controller_ros_args(robot_name, arm_name):
    public_base = f'/{robot_name}/{arm_name}/integrated_cartesian_admittance_controller'
    topic_names = [
        'equilibrium_twist_cmd',
        'singular_values',
        'debug_twist',
        'filtered_wrench',
        'equilibrium_pose',
        'target_pose',
        'collision_min_clearance',
        'collision_status',
        'collision_markers',
    ]
    remaps = ' '.join(
        f'-r ~/{topic}:={public_base}/{topic}' for topic in topic_names
    )
    return f'--ros-args {remaps}'


def make_arm_controller_spawners(context, *args, **kwargs):
    if LaunchConfiguration('load_arm_controllers').perform(context) != 'true':
        return []

    use_integrated = (
        LaunchConfiguration('use_integrated_cartesian_admittance_controller').perform(context)
        == 'true'
    )
    integrated_active = (
        LaunchConfiguration('integrated_controller_initial_active').perform(context) == 'true'
    )
    forward_inactive = use_integrated and integrated_active
    actions = [
        controller_spawner('forward_velocity_controller_l', inactive=forward_inactive),
        controller_spawner('forward_velocity_controller_r', inactive=forward_inactive),
        controller_spawner('joint_trajectory_controller_l', inactive=True),
        controller_spawner('joint_trajectory_controller_r', inactive=True),
        controller_spawner('joint_trajectory_controller_lift_l', inactive=True),
        controller_spawner('joint_trajectory_controller_lift_r', inactive=True),
    ]
    if use_integrated:
        robot_name = LaunchConfiguration('robot_name').perform(context)
        actions.extend([
            controller_spawner(
                'UR10_l_integrated_cartesian_admittance_controller',
                inactive=not integrated_active,
                controller_ros_args=integrated_controller_ros_args(robot_name, 'UR10_l'),
            ),
            controller_spawner(
                'UR10_r_integrated_cartesian_admittance_controller',
                inactive=not integrated_active,
                controller_ros_args=integrated_controller_ros_args(robot_name, 'UR10_r'),
            ),
        ])

    return [TimerAction(period=3.0, actions=actions)]


def moveit_proxy_arguments(robot_name, arm, *, include_lift, integrated_active):
    arguments = ['--robot-name', robot_name, '--arm', arm]
    if include_lift:
        arguments.append('--include-lift')
    if integrated_active:
        arm_name = f'UR10_{arm}'
        arguments.extend([
            '--velocity-controller',
            f'{arm_name}_integrated_cartesian_admittance_controller',
            '--velocity-command-topic',
            f'/{robot_name}/{arm_name}/integrated_cartesian_admittance_controller/'
            'equilibrium_twist_cmd',
            '--velocity-command-type',
            'twist_stamped',
            '--velocity-command-frame',
            f'{arm_name}/base_link',
        ])
    return arguments


def make_moveit_controller_proxy_spawners(context, *args, **kwargs):
    if LaunchConfiguration('auto_switch_arm_controllers').perform(context) != 'true':
        return []

    robot_name = LaunchConfiguration('robot_name').perform(context)
    integrated_active = (
        LaunchConfiguration('use_integrated_cartesian_admittance_controller').perform(context)
        == 'true'
        and LaunchConfiguration('integrated_controller_initial_active').perform(context) == 'true'
    )
    actions = []
    for arm in ('l', 'r'):
        actions.append(
            Node(
                package='mur_launch_sim',
                executable='moveit_trajectory_controller_proxy.py',
                name=f'{robot_name}_moveit_controller_proxy_{arm}',
                arguments=moveit_proxy_arguments(
                    robot_name, arm, include_lift=False, integrated_active=integrated_active
                ),
                output='screen',
            )
        )
        actions.append(
            Node(
                package='mur_launch_sim',
                executable='moveit_trajectory_controller_proxy.py',
                name=f'{robot_name}_moveit_lift_controller_proxy_{arm}',
                arguments=moveit_proxy_arguments(
                    robot_name, arm, include_lift=True, integrated_active=integrated_active
                ),
                output='screen',
            )
        )

    return [TimerAction(period=3.0, actions=actions)]


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
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('lidar_bridge', default_value='true'),
        DeclareLaunchArgument('start_controller_manager', default_value='false'),
        DeclareLaunchArgument('load_controllers', default_value='true'),
        DeclareLaunchArgument('laser_merger', default_value='true'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('fake_localization', default_value='false'),
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
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(mur_moveit_path, 'config', 'mur620a.rviz'),
        ),
        DeclareLaunchArgument('launch_servo', default_value='false'),
        DeclareLaunchArgument('auto_switch_arm_controllers', default_value='true'),
        DeclareLaunchArgument('launch_jparse_idk', default_value='true'),
        DeclareLaunchArgument('publish_global_moveit_descriptions', default_value='true'),
        DeclareLaunchArgument('publish_tf_alias', default_value='true'),
        DeclareLaunchArgument('tf_topic', default_value='/tf'),
        DeclareLaunchArgument('tf_static_topic', default_value='/tf_static'),
        DeclareLaunchArgument('virtual_joint_parent_frame', default_value=''),
        DeclareLaunchArgument('ur_type', default_value='ur10e'),
    ] + integrated_launch_arguments()

    base_arguments = {
        'robot_name': LaunchConfiguration('robot_name'),
        'world': LaunchConfiguration('world'),
        'x': LaunchConfiguration('x'),
        'y': LaunchConfiguration('y'),
        'z': LaunchConfiguration('z'),
        'Y': LaunchConfiguration('Y'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'include_gz': LaunchConfiguration('include_gz'),
        'gazebo_gui': LaunchConfiguration('gazebo_gui'),
        'lidar_bridge': LaunchConfiguration('lidar_bridge'),
        'start_controller_manager': LaunchConfiguration('start_controller_manager'),
        'load_controllers': LaunchConfiguration('load_controllers'),
        'laser_merger': LaunchConfiguration('laser_merger'),
        'localization': LaunchConfiguration('localization'),
        'fake_localization': LaunchConfiguration('fake_localization'),
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
    base_arguments.update(integrated_base_arguments())

    base_arguments['map'] = LaunchConfiguration('map')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mur_base_launch),
        launch_arguments=base_arguments.items(),
    )

    arm_controllers = OpaqueFunction(function=make_arm_controller_spawners)

    moveit_descriptions = Node(
        package='mur_launch_sim',
        executable='publish_moveit_descriptions.py',
        name=[LaunchConfiguration('robot_name'), '_moveit_descriptions'],
        condition=IfCondition(LaunchConfiguration('launch_moveit')),
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_global_topics': LaunchConfiguration(
                'publish_global_moveit_descriptions'
            ),
            'publish_namespaced_topics': 'true',
        }],
        output='screen',
    )

    arm_controller_switchers = OpaqueFunction(
        function=make_moveit_controller_proxy_spawners
    )

    jparse_idk = TimerAction(
        period=4.0,
        condition=IfCondition(LaunchConfiguration('launch_jparse_idk')),
        actions=[
            Node(
                package='mur_control',
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
                package='mur_control',
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
                package='mur_control',
                executable='jparse_move_action_server.py',
                name=[LaunchConfiguration('robot_name'), '_jparse_move_l'],
                arguments=[
                    '--robot-name', LaunchConfiguration('robot_name'),
                    '--arm', 'l',
                ],
                output='screen',
            ),
            Node(
                package='mur_control',
                executable='jparse_move_action_server.py',
                name=[LaunchConfiguration('robot_name'), '_jparse_move_r'],
                arguments=[
                    '--robot-name', LaunchConfiguration('robot_name'),
                    '--arm', 'r',
                ],
                output='screen',
            ),
            Node(
                package='mur_control',
                executable='jparse_simple_goal.py',
                name=[LaunchConfiguration('robot_name'), '_jparse_simple_goal_l'],
                arguments=[
                    '--robot-name', LaunchConfiguration('robot_name'),
                    '--arm', 'l',
                ],
                output='screen',
            ),
            Node(
                package='mur_control',
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
            'rviz_config': LaunchConfiguration('rviz_config'),
            'controller_namespace': LaunchConfiguration('robot_name'),
            'publish_tf_alias': LaunchConfiguration('publish_tf_alias'),
            'tf_topic': LaunchConfiguration('tf_topic'),
            'tf_static_topic': LaunchConfiguration('tf_static_topic'),
            'virtual_joint_parent_frame': LaunchConfiguration('virtual_joint_parent_frame'),
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
