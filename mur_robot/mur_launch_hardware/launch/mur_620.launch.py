"""Generic hardware launch for a MUR620 with two UR arms."""

from copy import deepcopy
import importlib.util
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    mur_launch_hardware = get_package_share_directory('mur_launch_hardware')
    mur_moveit_config = get_package_share_directory('mur_moveit_config')

    return [
        DeclareLaunchArgument('robot_name', default_value='mur620'),
        DeclareLaunchArgument('robot_profile', default_value='mur620d'),
        DeclareLaunchArgument(
            'robot_profile_file',
            default_value=os.path.join(mur_launch_hardware, 'config', 'mur_robot_profiles.yaml'),
        ),
        DeclareLaunchArgument('ur_type', default_value='ur10'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_mur_tf', default_value='true'),
        DeclareLaunchArgument('use_arms', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_lift', default_value=''),
        DeclareLaunchArgument('use_simple_collisions', default_value='false'),
        DeclareLaunchArgument('use_simple_visuals', default_value='false'),
        DeclareLaunchArgument('use_high_quality_visuals', default_value='false'),
        DeclareLaunchArgument('use_base_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_top_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_wheel_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_caster_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_lift_visual_mesh', default_value='false'),
        DeclareLaunchArgument('launch_lift_l', default_value=''),
        DeclareLaunchArgument('launch_lift_r', default_value=''),
        DeclareLaunchArgument('publish_fake_mir_wheel_joints', default_value='true'),
        DeclareLaunchArgument('fake_mir_wheel_joint_frequency', default_value='10.0'),
        DeclareLaunchArgument('launch_mir', default_value='false'),
        DeclareLaunchArgument('mir_hostname', default_value='192.168.12.20'),
        DeclareLaunchArgument('mir_port', default_value='9090'),
        DeclareLaunchArgument('mir_type', default_value='mir_600'),
        DeclareLaunchArgument(
            'mir_enabled_pub_topics',
            default_value='b_raw_scan b_scan f_raw_scan f_scan scan robot_pose map map_metadata odom odom_enc tf tf_static',
            description='Space separated ROS1 MiR topics to bridge into ROS 2 when launch_mir is true.',
        ),
        DeclareLaunchArgument('launch_bms', default_value='true'),
        DeclareLaunchArgument(
            'battery_node_id',
            default_value='',
            description='BMS CAN node id. Empty uses the selected robot profile value.',
        ),
        DeclareLaunchArgument('bms_can_interface', default_value='can0'),
        DeclareLaunchArgument('bms_can_bitrate', default_value='250000'),
        DeclareLaunchArgument('bms_publish_frequency', default_value='1.0'),
        DeclareLaunchArgument('bms_response_timeout', default_value='0.8'),
        DeclareLaunchArgument('bms_configure_can_interface', default_value='false'),
        # Matches the ROS 1 hardware launch wiring for the MUR620 lifts.
        DeclareLaunchArgument('lift_port_l', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lift_port_r', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lift_baud', default_value='38400'),
        DeclareLaunchArgument('lift_timeout', default_value='1000'),
        DeclareLaunchArgument('lift_joint_count', default_value='2'),
        DeclareLaunchArgument('lift_conversion', default_value='3225.0'),
        DeclareLaunchArgument('lift_rated_effort', default_value='2000.0'),
        DeclareLaunchArgument('lift_tolerance', default_value='0.002'),
        DeclareLaunchArgument('lift_frequency', default_value='10.0'),
        DeclareLaunchArgument('lift_allow_zero_command', default_value='false'),
        DeclareLaunchArgument('lift_position_multiplier', default_value='1.0'),
        DeclareLaunchArgument('use_laser_visual_mesh', default_value='false'),
        DeclareLaunchArgument(
            'ur_l_xyz',
            default_value='',
        ),
        DeclareLaunchArgument('ur_l_rpy', default_value=''),
        DeclareLaunchArgument(
            'ur_r_xyz',
            default_value='',
        ),
        DeclareLaunchArgument('ur_r_rpy', default_value=''),
        DeclareLaunchArgument('launch_ur_l', default_value='true'),
        DeclareLaunchArgument('launch_ur_r', default_value='true'),
        DeclareLaunchArgument('robot_ip_l', default_value='UR10_l'),
        DeclareLaunchArgument('robot_ip_r', default_value='UR10_r'),
        DeclareLaunchArgument('reverse_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('headless_mode', default_value='false'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false'),
        DeclareLaunchArgument('launch_dashboard_client', default_value='true'),
        DeclareLaunchArgument('auto_start_urs', default_value='true'),
        DeclareLaunchArgument('ur_startup_delay', default_value='3.0'),
        DeclareLaunchArgument('ur_startup_wait_timeout', default_value='60.0'),
        DeclareLaunchArgument('ur_startup_stop_program', default_value='true'),
        DeclareLaunchArgument('ur_startup_play_program', default_value='true'),
        DeclareLaunchArgument('ur_startup_verify_program_running', default_value='true'),
        DeclareLaunchArgument('initial_joint_controller', default_value='forward_velocity_controller'),
        DeclareLaunchArgument('activate_joint_controller', default_value='true'),
        DeclareLaunchArgument('launch_trajectory_until_node', default_value='false'),
        DeclareLaunchArgument('controller_spawner_timeout', default_value='10'),
        DeclareLaunchArgument('launch_jparse_idk', default_value='true'),
        DeclareLaunchArgument('jparse_startup_delay', default_value='5.0'),
        DeclareLaunchArgument('jparse_rate_hz', default_value='500.0'),
        DeclareLaunchArgument('jparse_command_timeout', default_value='0.12'),
        DeclareLaunchArgument('jparse_inverse_mode', default_value='jparse'),
        DeclareLaunchArgument('jparse_damping', default_value='0.03'),
        DeclareLaunchArgument('jparse_max_joint_velocity', default_value='0.6'),
        DeclareLaunchArgument('jparse_max_linear_velocity', default_value='0.12'),
        DeclareLaunchArgument('jparse_max_angular_velocity', default_value='0.5'),
        DeclareLaunchArgument('launch_cartesian_admittance', default_value='false'),
        DeclareLaunchArgument('cartesian_admittance_rate_hz', default_value='500.0'),
        DeclareLaunchArgument('cartesian_admittance_command_timeout', default_value='0.12'),
        DeclareLaunchArgument('cartesian_admittance_wrench_timeout', default_value='0.5'),
        DeclareLaunchArgument('cartesian_admittance_require_wrench', default_value='true'),
        DeclareLaunchArgument('cartesian_admittance_wrench_bias_duration', default_value='1.0'),
        DeclareLaunchArgument('cartesian_admittance_wrench_filter_alpha', default_value='0.02'),
        DeclareLaunchArgument('cartesian_admittance_max_linear_velocity', default_value='0.10'),
        DeclareLaunchArgument('cartesian_admittance_max_angular_velocity', default_value='0.35'),
        DeclareLaunchArgument('use_integrated_cartesian_admittance_controller', default_value='false'),
        DeclareLaunchArgument('integrated_controller_initial_active', default_value='false'),
        DeclareLaunchArgument('launch_integrated_cartesian_move_action', default_value='true'),
        DeclareLaunchArgument('integrated_controller_use_ft_sensor', default_value='false'),
        DeclareLaunchArgument('integrated_controller_require_wrench', default_value='false'),
        DeclareLaunchArgument('integrated_controller_command_timeout', default_value='0.12'),
        DeclareLaunchArgument('integrated_controller_wrench_timeout', default_value='0.5'),
        DeclareLaunchArgument('integrated_controller_wrench_bias_duration', default_value='1.0'),
        DeclareLaunchArgument('integrated_controller_wrench_filter_alpha', default_value='0.20'),
        DeclareLaunchArgument('integrated_controller_wrench_in_tcp_frame', default_value='true'),
        DeclareLaunchArgument(
            'integrated_controller_wrench_sign',
            default_value='1.0 1.0 1.0 1.0 1.0 1.0',
        ),
        DeclareLaunchArgument('integrated_controller_force_deadband', default_value='0.2'),
        DeclareLaunchArgument('integrated_controller_torque_deadband', default_value='0.05'),
        DeclareLaunchArgument(
            'integrated_controller_admittance',
            default_value='0.0012 0.0012 0.0013 0.0 0.0 0.0',
        ),
        DeclareLaunchArgument(
            'integrated_controller_wrench_twist_gain',
            default_value='0.0007 0.0007 0.0006 0.0 0.0 0.0',
        ),
        DeclareLaunchArgument(
            'integrated_controller_pose_error_gain',
            default_value='1.9 1.9 1.7 0.75 0.75 0.75',
        ),
        DeclareLaunchArgument('integrated_controller_max_linear_velocity', default_value='0.16'),
        DeclareLaunchArgument('integrated_controller_max_angular_velocity', default_value='0.55'),
        DeclareLaunchArgument('integrated_controller_max_joint_velocity', default_value='0.6'),
        DeclareLaunchArgument('integrated_controller_max_joint_acceleration', default_value='1.4'),
        DeclareLaunchArgument('integrated_controller_max_joint_jerk', default_value='6.0'),
        DeclareLaunchArgument('integrated_controller_joint_limit_margin', default_value='0.02'),
        DeclareLaunchArgument('integrated_controller_preserve_command_direction', default_value='true'),
        DeclareLaunchArgument('integrated_controller_immediate_zero_on_zero_command', default_value='true'),
        DeclareLaunchArgument('integrated_controller_zero_command_deadband', default_value='1.0e-5'),
        DeclareLaunchArgument('integrated_controller_reset_equilibrium_on_zero_command', default_value='auto'),
        DeclareLaunchArgument('integrated_controller_enable_collision_avoidance', default_value='true'),
        DeclareLaunchArgument('integrated_controller_collision_common_link', default_value='base_link'),
        DeclareLaunchArgument('integrated_controller_collision_joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('integrated_controller_collision_joint_state_timeout', default_value='0.1'),
        DeclareLaunchArgument('integrated_controller_collision_sample_spacing', default_value='0.08'),
        DeclareLaunchArgument('integrated_controller_collision_sphere_radius', default_value='0.04'),
        DeclareLaunchArgument('integrated_controller_collision_activation_clearance', default_value='0.08'),
        DeclareLaunchArgument('integrated_controller_collision_stop_clearance', default_value='0.035'),
        DeclareLaunchArgument('integrated_controller_collision_fail_safe_stop', default_value='true'),
        DeclareLaunchArgument(
            'integrated_controller_collision_forbidden_boxes',
            default_value='default',
            description=(
                "Semicolon-separated forbidden boxes in collision_common_link as "
                "name:cx,cy,cz:sx,sy,sz. 'default' blocks the MiR chassis/deck "
                "and the opposite lift column for each arm; empty disables these boxes."
            ),
        ),
        DeclareLaunchArgument('integrated_controller_publish_collision_markers', default_value='false'),
        DeclareLaunchArgument('integrated_controller_collision_marker_publish_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('launch_moveit', default_value='false'),
        DeclareLaunchArgument('launch_moveit_rviz', default_value='false'),
        DeclareLaunchArgument('moveit_rviz_delay', default_value='5.0'),
        DeclareLaunchArgument(
            'moveit_rviz_config',
            default_value=os.path.join(mur_moveit_config, 'config', 'moveit.rviz'),
        ),
        DeclareLaunchArgument('auto_switch_moveit_controllers', default_value='true'),
        DeclareLaunchArgument('moveit_controller_proxy_delay', default_value='8.0'),
        DeclareLaunchArgument(
            'moveit_hardware_trajectory_controller',
            default_value='scaled_joint_trajectory_controller',
        ),
        DeclareLaunchArgument('moveit_controller_switch_timeout', default_value='5.0'),
        DeclareLaunchArgument('moveit_trajectory_action_timeout', default_value='20.0'),
        DeclareLaunchArgument('moveit_post_result_settle_sec', default_value='0.35'),
        DeclareLaunchArgument('moveit_goal_reached_tolerance', default_value='0.03'),
        DeclareLaunchArgument('moveit_default_velocity_scaling', default_value='0.25'),
        DeclareLaunchArgument('moveit_default_acceleration_scaling', default_value='0.15'),
        DeclareLaunchArgument('moveit_velocity_controller', default_value='forward_velocity_controller'),
        DeclareLaunchArgument('launch_arm_velocity_safety', default_value='true'),
        DeclareLaunchArgument('arm_velocity_safety_rate_hz', default_value='500.0'),
        DeclareLaunchArgument('arm_velocity_safety_command_timeout', default_value='0.15'),
        DeclareLaunchArgument('arm_velocity_safety_max_joint_velocity', default_value='0.6'),
        DeclareLaunchArgument('arm_velocity_safety_max_joint_acceleration', default_value='0.4'),
        DeclareLaunchArgument('arm_velocity_safety_max_joint_jerk', default_value='1.0'),
        DeclareLaunchArgument('arm_velocity_safety_preserve_command_direction', default_value='true'),
        DeclareLaunchArgument('arm_velocity_safety_immediate_zero_on_zero_command', default_value='true'),
        DeclareLaunchArgument('arm_velocity_safety_zero_command_deadband', default_value='1.0e-5'),
        DeclareLaunchArgument('arm_velocity_safety_joint_limit_margin', default_value='0.02'),
        DeclareLaunchArgument('arm_collision_avoidance', default_value='true'),
        DeclareLaunchArgument('arm_collision_stop_distance', default_value='0.14'),
        DeclareLaunchArgument('arm_collision_release_distance', default_value='0.18'),
        DeclareLaunchArgument('safety_limits', default_value='true'),
        DeclareLaunchArgument('safety_pos_margin', default_value='0.15'),
        DeclareLaunchArgument('safety_k_position', default_value='20'),
        DeclareLaunchArgument('use_tool_communication', default_value='false'),
        DeclareLaunchArgument('tool_parity', default_value='0'),
        DeclareLaunchArgument('tool_baud_rate', default_value='115200'),
        DeclareLaunchArgument('tool_stop_bits', default_value='1'),
        DeclareLaunchArgument('tool_rx_idle_chars', default_value='1.5'),
        DeclareLaunchArgument('tool_tx_idle_chars', default_value='3.5'),
        DeclareLaunchArgument('tool_voltage', default_value='0'),
        DeclareLaunchArgument('tool_device_name_l', default_value='/tmp/ttyUR_l'),
        DeclareLaunchArgument('tool_device_name_r', default_value='/tmp/ttyUR_r'),
        DeclareLaunchArgument('tool_tcp_port_l', default_value='54321'),
        DeclareLaunchArgument('tool_tcp_port_r', default_value='54322'),
        DeclareLaunchArgument('reverse_port_l', default_value='50005'),
        DeclareLaunchArgument('script_sender_port_l', default_value='50006'),
        DeclareLaunchArgument('trajectory_port_l', default_value='50007'),
        DeclareLaunchArgument('script_command_port_l', default_value='50008'),
        DeclareLaunchArgument('reverse_port_r', default_value='50001'),
        DeclareLaunchArgument('script_sender_port_r', default_value='50002'),
        DeclareLaunchArgument('trajectory_port_r', default_value='50003'),
        DeclareLaunchArgument('script_command_port_r', default_value='50004'),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('ur_robot_driver'), 'config', 'ur_controllers.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'kinematics_params_file_l',
            default_value='',
        ),
        DeclareLaunchArgument(
            'kinematics_params_file_r',
            default_value='',
        ),
    ]


def ur_description_arguments(side, kinematics_params_file):
    return {
        'ur_type': LaunchConfiguration('ur_type'),
        'robot_ip': LaunchConfiguration(f'robot_ip_{side}'),
        'safety_limits': LaunchConfiguration('safety_limits'),
        'safety_pos_margin': LaunchConfiguration('safety_pos_margin'),
        'safety_k_position': LaunchConfiguration('safety_k_position'),
        'kinematics_params_file': kinematics_params_file,
        'tf_prefix': f'UR10_{side}/',
        'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
        'mock_sensor_commands': LaunchConfiguration('mock_sensor_commands'),
        'headless_mode': LaunchConfiguration('headless_mode'),
        'use_tool_communication': LaunchConfiguration('use_tool_communication'),
        'tool_parity': LaunchConfiguration('tool_parity'),
        'tool_baud_rate': LaunchConfiguration('tool_baud_rate'),
        'tool_stop_bits': LaunchConfiguration('tool_stop_bits'),
        'tool_rx_idle_chars': LaunchConfiguration('tool_rx_idle_chars'),
        'tool_tx_idle_chars': LaunchConfiguration('tool_tx_idle_chars'),
        'tool_device_name': LaunchConfiguration(f'tool_device_name_{side}'),
        'tool_tcp_port': LaunchConfiguration(f'tool_tcp_port_{side}'),
        'tool_voltage': LaunchConfiguration('tool_voltage'),
        'reverse_ip': LaunchConfiguration('reverse_ip'),
        'reverse_port': LaunchConfiguration(f'reverse_port_{side}'),
        'script_sender_port': LaunchConfiguration(f'script_sender_port_{side}'),
        'trajectory_port': LaunchConfiguration(f'trajectory_port_{side}'),
        'script_command_port': LaunchConfiguration(f'script_command_port_{side}'),
    }


def controller_spawner(namespace, controllers, *, active=True):
    inactive_flags = [] if active else ['--inactive']
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            '--controller-manager',
            f'/{namespace}/controller_manager',
            '--controller-manager-timeout',
            LaunchConfiguration('controller_spawner_timeout'),
        ] + inactive_flags + controllers,
        output='screen',
    )


def make_arm_controllers_file(source_file, robot_name, arm_name, integrated_controller_params):
    with open(source_file, 'r', encoding='utf-8') as config:
        contents = config.read()

    contents = contents.replace('$(var tf_prefix)', f'{arm_name}/')
    loaded = yaml.safe_load(contents)

    namespace = f'{robot_name}/{arm_name}'
    tcp_pose_params = loaded.get('tcp_pose_broadcaster', {}).get('ros__parameters', {})
    if isinstance(tcp_pose_params, dict):
        tcp_pose_params['frame_id'] = f'{namespace}/base'
        tcp_pose_tf = tcp_pose_params.setdefault('tf', {})
        tcp_pose_tf['child_frame_id'] = f'{namespace}/tool0_controller'

    namespaced = deepcopy(loaded)
    controller_manager_config = deepcopy(loaded.get('controller_manager', {}))
    namespaced[f'/{namespace}/controller_manager'] = controller_manager_config

    controller_params = controller_manager_config.get('ros__parameters', {})
    controller_params['integrated_cartesian_admittance_controller'] = {
        'type': 'mur_control/IntegratedCartesianAdmittanceController',
    }
    namespaced['integrated_cartesian_admittance_controller'] = {
        'ros__parameters': integrated_controller_params,
    }
    for controller_name, controller_config in controller_params.items():
        if not isinstance(controller_config, dict) or 'type' not in controller_config:
            continue
        namespaced[f'/{namespace}/{controller_name}'] = deepcopy(
            namespaced.get(controller_name, loaded.get(controller_name, {'ros__parameters': {}}))
        )

    out_dir = os.path.join(tempfile.gettempdir(), 'mur_launch_hardware')
    os.makedirs(out_dir, exist_ok=True)
    out_file = os.path.join(out_dir, f'{robot_name}_{arm_name}_ur_controllers.yaml')

    with open(out_file, 'w', encoding='utf-8') as config:
        yaml.safe_dump(namespaced, config, sort_keys=False)

    return out_file


def make_ur_driver(side, robot_name, controllers_file, update_rate_config_file, kinematics_params_file):
    arm_name = f'UR10_{side}'
    namespace = f'{robot_name}/{arm_name}'
    ur_robot_driver_share = get_package_share_directory('ur_robot_driver')
    ur_rsp_launch = os.path.join(ur_robot_driver_share, 'launch', 'ur_rsp.launch.py')

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    launch_dashboard_client = LaunchConfiguration('launch_dashboard_client')
    use_tool_communication = LaunchConfiguration('use_tool_communication')
    activate_joint_controller = LaunchConfiguration('activate_joint_controller')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller').perform

    controllers_active = [
        'joint_state_broadcaster',
        'io_and_status_controller',
        'speed_scaling_state_broadcaster',
        'force_torque_sensor_broadcaster',
        'tcp_pose_broadcaster',
        'ur_configuration_controller',
    ]
    controllers_inactive = [
        'scaled_joint_trajectory_controller',
        'joint_trajectory_controller',
        'forward_velocity_controller',
        'forward_position_controller',
        'forward_effort_controller',
        'force_mode_controller',
        'passthrough_trajectory_controller',
        'freedrive_mode_controller',
        'tool_contact_controller',
    ]

    selected_controller = initial_joint_controller

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[
            update_rate_config_file,
            controllers_file,
        ],
        remappings=[
            ('joint_states', '/joint_states'),
        ],
        output='screen',
    )

    dashboard_client_node = Node(
        package='ur_robot_driver',
        executable='dashboard_client',
        name='dashboard_client',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        condition=IfCondition(AndSubstitution(launch_dashboard_client, NotSubstitution(use_mock_hardware))),
        parameters=[
            {'robot_ip': LaunchConfiguration(f'robot_ip_{side}')},
            {'receive_timeout': 20.0},
        ],
    )

    robot_state_helper_node = Node(
        package='ur_robot_driver',
        executable='robot_state_helper',
        name='ur_robot_state_helper',
        namespace=namespace,
        output='screen',
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {'headless_mode': LaunchConfiguration('headless_mode')},
            {'robot_ip': LaunchConfiguration(f'robot_ip_{side}')},
        ],
    )

    tool_communication_node = Node(
        package='ur_robot_driver',
        executable='tool_communication.py',
        name='ur_tool_comm',
        namespace=namespace,
        output='screen',
        condition=IfCondition(use_tool_communication),
        parameters=[{
            'robot_ip': LaunchConfiguration(f'robot_ip_{side}'),
            'tcp_port': LaunchConfiguration(f'tool_tcp_port_{side}'),
            'device_name': LaunchConfiguration(f'tool_device_name_{side}'),
        }],
    )

    urscript_interface = Node(
        package='ur_robot_driver',
        executable='urscript_interface',
        namespace=namespace,
        parameters=[{'robot_ip': LaunchConfiguration(f'robot_ip_{side}')}],
        output='screen',
        condition=UnlessCondition(use_mock_hardware),
    )

    controller_stopper_node = Node(
        package='ur_robot_driver',
        executable='controller_stopper_node',
        name='controller_stopper',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(use_mock_hardware),
        parameters=[
            {'headless_mode': LaunchConfiguration('headless_mode')},
            {'joint_controller_active': activate_joint_controller},
            {'consistent_controllers': [
                'io_and_status_controller',
                'force_torque_sensor_broadcaster',
                'joint_state_broadcaster',
                'speed_scaling_state_broadcaster',
                'tcp_pose_broadcaster',
                'ur_configuration_controller',
                'forward_velocity_controller',
                'integrated_cartesian_admittance_controller',
                'freedrive_mode_controller',
                'scaled_joint_trajectory_controller',
                'joint_trajectory_controller',
            ]},
        ],
    )

    trajectory_until_node = Node(
        package='ur_robot_driver',
        executable='trajectory_until_node',
        name='trajectory_until_node',
        namespace=namespace,
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_trajectory_until_node')),
        parameters=[{'motion_controller': LaunchConfiguration('initial_joint_controller')}],
    )

    startup_enable_node = TimerAction(
        period=LaunchConfiguration('ur_startup_delay'),
        actions=[
            Node(
                package='mur_launch_hardware',
                executable='ur_startup_enable.py',
                name=f'{arm_name}_startup_enable',
                output='screen',
                condition=IfCondition(AndSubstitution(
                    LaunchConfiguration('auto_start_urs'),
                    NotSubstitution(use_mock_hardware),
                )),
                parameters=[{
                    'arm_namespace': f'/{namespace}',
                    'wait_timeout': LaunchConfiguration('ur_startup_wait_timeout'),
                    'target_robot_mode': 7,
                    'stop_program': LaunchConfiguration('ur_startup_stop_program'),
                    'play_program': LaunchConfiguration('ur_startup_play_program'),
                    'verify_program_running': LaunchConfiguration('ur_startup_verify_program_running'),
                }],
            ),
        ],
    )

    ur_rsp = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst=f'/{namespace}/unused_tf'),
        SetRemap(src='/tf_static', dst=f'/{namespace}/unused_tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_rsp_launch),
            launch_arguments=ur_description_arguments(side, kinematics_params_file).items(),
        ),
    ])

    def add_selected_controller(context):
        selected = selected_controller(context)
        active = list(controllers_active)
        inactive = list(controllers_inactive)
        use_integrated = LaunchConfiguration('use_integrated_cartesian_admittance_controller').perform(context) == 'true'
        integrated_active = (
            LaunchConfiguration('integrated_controller_initial_active').perform(context) == 'true'
        )
        if activate_joint_controller.perform(context) == 'true' and not (use_integrated and integrated_active):
            active.append(selected)
            if selected in inactive:
                inactive.remove(selected)
        if use_integrated:
            inactive.append('integrated_cartesian_admittance_controller')
            if integrated_active:
                active.append('integrated_cartesian_admittance_controller')
                inactive.remove('integrated_cartesian_admittance_controller')
                if 'forward_velocity_controller' in active:
                    active.remove('forward_velocity_controller')
                if 'forward_velocity_controller' not in inactive:
                    inactive.append('forward_velocity_controller')
        if use_mock_hardware.perform(context) == 'true' and 'tcp_pose_broadcaster' in active:
            active.remove('tcp_pose_broadcaster')

        active_spawner = controller_spawner(namespace, active)
        inactive_spawner = controller_spawner(namespace, inactive, active=False)
        startup_after_spawners = RegisterEventHandler(
            OnProcessExit(
                target_action=inactive_spawner,
                on_exit=[startup_enable_node],
            )
        )

        return [
            active_spawner,
            inactive_spawner,
            startup_after_spawners,
        ]

    return GroupAction(
        actions=[
            control_node,
            dashboard_client_node,
            robot_state_helper_node,
            tool_communication_node,
            urscript_interface,
            controller_stopper_node,
            ur_rsp,
            trajectory_until_node,
            OpaqueFunction(function=add_selected_controller),
        ],
        condition=IfCondition(LaunchConfiguration(f'launch_ur_{side}')),
    )


def make_lift_driver(side, robot_name, launch_condition):
    namespace = f'{robot_name}/ewellix_lift_{side}'

    return GroupAction(
        actions=[
            Node(
                package='ewellix_driver',
                executable='ewellix_node',
                name='ewellix_node',
                namespace=namespace,
                parameters=[{
                    'joint_count': LaunchConfiguration('lift_joint_count'),
                    'port': LaunchConfiguration(f'lift_port_{side}'),
                    'baud': LaunchConfiguration('lift_baud'),
                    'timeout': LaunchConfiguration('lift_timeout'),
                    'conversion': LaunchConfiguration('lift_conversion'),
                    'rated_effort': LaunchConfiguration('lift_rated_effort'),
                    'tolerance': LaunchConfiguration('lift_tolerance'),
                    'frequency': LaunchConfiguration('lift_frequency'),
                    'allow_zero_command': LaunchConfiguration('lift_allow_zero_command'),
                }],
                output='screen',
            ),
        ],
        condition=IfCondition(launch_condition),
    )


def make_lift_joint_state_bridge(robot_name, launch_condition):
    return Node(
        package='mur_launch_hardware',
        executable='ewellix_dual_state_to_joint_state.py',
        name='ewellix_lift_joint_state_bridge',
        namespace=robot_name,
        parameters=[{
            'robot_name': robot_name,
            'joint_count': LaunchConfiguration('lift_joint_count'),
            'conversion': LaunchConfiguration('lift_conversion'),
            'position_multiplier': LaunchConfiguration('lift_position_multiplier'),
            'publish_frequency': LaunchConfiguration('lift_frequency'),
            'joint_states_topic': '/joint_states',
        }],
        condition=IfCondition(launch_condition),
        output='screen',
    )



def make_mir_hardware_launch(robot_name, use_sim_time):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mir_launch_hardware'),
                'launch',
                'mir_hardware_launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('launch_mir')),
        launch_arguments={
            'namespace': robot_name,
            'use_sim_time': 'true' if use_sim_time else 'false',
            'mir_hostname': LaunchConfiguration('mir_hostname'),
            'mir_port': LaunchConfiguration('mir_port'),
            'mir_type': LaunchConfiguration('mir_type'),
            'enabled_pub_topics': LaunchConfiguration('mir_enabled_pub_topics'),
            'robot_state_publisher_enabled': 'false',
        }.items(),
    )

def make_fake_mir_wheel_joint_publisher(robot_name):
    return Node(
        package='mur_launch_hardware',
        executable='fake_mir_wheel_joint_states.py',
        name='fake_mir_wheel_joint_states',
        namespace=robot_name,
        parameters=[{
            'publish_frequency': LaunchConfiguration('fake_mir_wheel_joint_frequency'),
            'joint_states_topic': '/joint_states',
        }],
        condition=IfCondition(LaunchConfiguration('publish_fake_mir_wheel_joints')),
        output='screen',
    )


def make_bms_can_node(robot_name, battery_node_id):
    return Node(
        package='mur_launch_hardware',
        executable='bms_can_node.py',
        name='bms_can_node',
        namespace=robot_name,
        parameters=[{
            'battery_node_id': battery_node_id,
            'can_interface': LaunchConfiguration('bms_can_interface'),
            'can_bitrate': LaunchConfiguration('bms_can_bitrate'),
            'publish_frequency': LaunchConfiguration('bms_publish_frequency'),
            'response_timeout': LaunchConfiguration('bms_response_timeout'),
            'configure_can_interface': LaunchConfiguration('bms_configure_can_interface'),
            'soc_topic': 'bms_status/SOC',
            'battery_state_topic': 'bms_status/battery_state',
        }],
        condition=IfCondition(LaunchConfiguration('launch_bms')),
        output='screen',
    )


def make_arm_velocity_safety_node(robot_name):
    return Node(
        package='mur_control',
        executable='arm_velocity_safety_node',
        name=f'{robot_name}_arm_velocity_safety',
        parameters=[{
            'robot_name': robot_name,
            'joint_states_topic': '/joint_states',
            'rate_hz': LaunchConfiguration('arm_velocity_safety_rate_hz'),
            'command_timeout': LaunchConfiguration('arm_velocity_safety_command_timeout'),
            'max_joint_velocity': LaunchConfiguration('arm_velocity_safety_max_joint_velocity'),
            'max_joint_acceleration': LaunchConfiguration('arm_velocity_safety_max_joint_acceleration'),
            'max_joint_jerk': LaunchConfiguration('arm_velocity_safety_max_joint_jerk'),
            'preserve_command_direction': LaunchConfiguration(
                'arm_velocity_safety_preserve_command_direction'
            ),
            'immediate_zero_on_zero_command': LaunchConfiguration(
                'arm_velocity_safety_immediate_zero_on_zero_command'
            ),
            'zero_command_deadband': LaunchConfiguration(
                'arm_velocity_safety_zero_command_deadband'
            ),
            'joint_limit_margin': LaunchConfiguration('arm_velocity_safety_joint_limit_margin'),
            'enable_collision_avoidance': LaunchConfiguration('arm_collision_avoidance'),
            'collision_stop_distance': LaunchConfiguration('arm_collision_stop_distance'),
            'collision_release_distance': LaunchConfiguration('arm_collision_release_distance'),
            'l_input_topic': f'/{robot_name}/UR10_l/safe_forward_velocity_controller/commands',
            'l_output_topic': f'/{robot_name}/UR10_l/forward_velocity_controller/commands',
            'r_input_topic': f'/{robot_name}/UR10_r/safe_forward_velocity_controller/commands',
            'r_output_topic': f'/{robot_name}/UR10_r/forward_velocity_controller/commands',
        }],
        condition=IfCondition(LaunchConfiguration('launch_arm_velocity_safety')),
        output='screen',
    )


def make_moveit_controller_proxies(robot_name):
    actions = []
    for side in ('l', 'r'):
        arm_name = f'UR10_{side}'
        actions.append(
            Node(
                package='mur_control',
                executable='moveit_trajectory_controller_proxy.py',
                name=f'{robot_name}_moveit_controller_proxy_{side}',
                arguments=[
                    '--robot-name', robot_name,
                    '--arm', side,
                    '--proxy-action',
                    f'/{robot_name}/moveit_joint_trajectory_controller_{side}/follow_joint_trajectory',
                    '--real-action',
                    [
                        f'/{robot_name}/{arm_name}/',
                        LaunchConfiguration('moveit_hardware_trajectory_controller'),
                        '/follow_joint_trajectory',
                    ],
                    '--controller-manager', f'/{robot_name}/{arm_name}/controller_manager',
                    '--velocity-controller', LaunchConfiguration('moveit_velocity_controller'),
                    '--trajectory-controller',
                    LaunchConfiguration('moveit_hardware_trajectory_controller'),
                    '--velocity-command-topic',
                    f'/{robot_name}/{arm_name}/safe_forward_velocity_controller/commands',
                    '--joint-states-topic', '/joint_states',
                    '--switch-timeout', LaunchConfiguration('moveit_controller_switch_timeout'),
                    '--action-timeout', LaunchConfiguration('moveit_trajectory_action_timeout'),
                    '--post-result-settle-sec', LaunchConfiguration('moveit_post_result_settle_sec'),
                    '--goal-reached-tolerance', LaunchConfiguration('moveit_goal_reached_tolerance'),
                ],
                output='screen',
            )
        )

    return TimerAction(
        period=LaunchConfiguration('moveit_controller_proxy_delay'),
        condition=IfCondition(AndSubstitution(
            LaunchConfiguration('launch_moveit'),
            LaunchConfiguration('auto_switch_moveit_controllers'),
        )),
        actions=actions,
    )


def make_moveit_launch(robot_name, home_custom_l_shoulder_pan, home_custom_r_shoulder_pan):
    mur_moveit_path = get_package_share_directory('mur_moveit_config')
    moveit_launch = os.path.join(mur_moveit_path, 'launch', 'ur_moveit.launch.py')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        condition=IfCondition(LaunchConfiguration('launch_moveit')),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'launch_servo': 'false',
            'launch_rviz': LaunchConfiguration('launch_moveit_rviz'),
            'rviz_delay': LaunchConfiguration('moveit_rviz_delay'),
            'rviz_config': LaunchConfiguration('moveit_rviz_config'),
            'controller_namespace': robot_name,
            'publish_tf_alias': 'true',
            'tf_topic': '/tf',
            'tf_static_topic': '/tf_static',
            'joint_states_topic': '/joint_states',
            'virtual_joint_parent_frame': f'{robot_name}/base_footprint',
            'default_velocity_scaling': LaunchConfiguration('moveit_default_velocity_scaling'),
            'default_acceleration_scaling': LaunchConfiguration('moveit_default_acceleration_scaling'),
            'home_custom_l_shoulder_pan': home_custom_l_shoulder_pan,
            'home_custom_r_shoulder_pan': home_custom_r_shoulder_pan,
        }.items(),
    )


def make_jparse_nodes(robot_name):
    actions = []
    for side in ('l', 'r'):
        arm_name = f'UR10_{side}'
        twist_topic = f'/{robot_name}/jparse_velocity_controller_{side}/twist_cmd'
        command_topic = f'/{robot_name}/{arm_name}/safe_forward_velocity_controller/commands'
        debug_topic = f'/{robot_name}/jparse_velocity_controller_{side}/debug_twist'

        actions.extend([
            Node(
                package='mur_control',
                executable='jparse_velocity_controller',
                name=f'{robot_name}_jparse_velocity_controller_{side}',
                parameters=[{
                    'robot_name': robot_name,
                    'arm': side,
                    'joint_states_topic': '/joint_states',
                    'command_topic': command_topic,
                    'debug_twist_topic': debug_topic,
                    'rate_hz': LaunchConfiguration('jparse_rate_hz'),
                    'command_timeout': LaunchConfiguration('jparse_command_timeout'),
                    'inverse_mode': LaunchConfiguration('jparse_inverse_mode'),
                    'damping': LaunchConfiguration('jparse_damping'),
                    'max_joint_velocity': LaunchConfiguration('jparse_max_joint_velocity'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
                remappings=[('~/twist_cmd', twist_topic)],
                output='screen',
            ),
            Node(
                package='mur_control',
                executable='jparse_move_action_server.py',
                name=f'{robot_name}_jparse_move_{side}',
                arguments=[
                    '--robot-name', robot_name,
                    '--arm', side,
                    '--twist-topic', twist_topic,
                    '--joint-velocity-topic', command_topic,
                    '--joint-states-topic', '/joint_states',
                    '--max-linear-velocity', LaunchConfiguration('jparse_max_linear_velocity'),
                    '--max-angular-velocity', LaunchConfiguration('jparse_max_angular_velocity'),
                    '--max-joint-velocity', LaunchConfiguration('jparse_max_joint_velocity'),
                ],
                output='screen',
            ),
            Node(
                package='mur_control',
                executable='jparse_simple_goal.py',
                name=f'{robot_name}_jparse_simple_goal_{side}',
                arguments=[
                    '--robot-name', robot_name,
                    '--arm', side,
                    '--max-linear-velocity', LaunchConfiguration('jparse_max_linear_velocity'),
                    '--max-angular-velocity', LaunchConfiguration('jparse_max_angular_velocity'),
                ],
                output='screen',
            ),
        ])

    return TimerAction(
        period=LaunchConfiguration('jparse_startup_delay'),
        actions=actions,
        condition=IfCondition(LaunchConfiguration('launch_jparse_idk')),
    )


def make_integrated_cartesian_move_action_nodes(robot_name):
    actions = []
    for side in ('l', 'r'):
        arm_name = f'UR10_{side}'
        actions.append(
            Node(
                package='mur_control',
                executable='integrated_cartesian_move_action_server.py',
                name=f'{robot_name}_integrated_cartesian_move_{side}',
                arguments=[
                    '--robot-name', robot_name,
                    '--arm', side,
                    '--action-name', f'/{robot_name}/jparse_move_{side}',
                    '--twist-topic',
                    f'/{robot_name}/{arm_name}/integrated_cartesian_admittance_controller/equilibrium_twist_cmd',
                    '--max-linear-velocity', LaunchConfiguration('integrated_controller_max_linear_velocity'),
                    '--max-angular-velocity', LaunchConfiguration('integrated_controller_max_angular_velocity'),
                ],
                condition=IfCondition(AndSubstitution(
                    LaunchConfiguration('use_integrated_cartesian_admittance_controller'),
                    LaunchConfiguration('launch_integrated_cartesian_move_action'),
                    LaunchConfiguration(f'launch_ur_{side}'),
                )),
                output='screen',
            )
        )

    return TimerAction(
        period=LaunchConfiguration('jparse_startup_delay'),
        actions=actions,
    )


def make_cartesian_admittance_nodes(robot_name):
    actions = []
    for side in ('l', 'r'):
        arm_name = f'UR10_{side}'
        actions.append(
            Node(
                package='mur_control',
                executable='cartesian_admittance_controller',
                name=f'{robot_name}_cartesian_admittance_controller_{side}',
                parameters=[{
                    'robot_name': robot_name,
                    'arm': side,
                    'tf_base_frame': f'{robot_name}/{arm_name}/base_link',
                    'tf_tcp_frame': f'{robot_name}/{arm_name}/tool0',
                    'command_frame': f'{arm_name}/base_link',
                    'input_twist_topic':
                        f'/{robot_name}/cartesian_admittance_controller_{side}/equilibrium_twist_cmd',
                    'output_twist_topic': f'/{robot_name}/jparse_velocity_controller_{side}/twist_cmd',
                    'wrench_topic':
                        f'/{robot_name}/{arm_name}/force_torque_sensor_broadcaster/ft_data',
                    'rate_hz': LaunchConfiguration('cartesian_admittance_rate_hz'),
                    'command_timeout': LaunchConfiguration('cartesian_admittance_command_timeout'),
                    'wrench_timeout': LaunchConfiguration('cartesian_admittance_wrench_timeout'),
                    'require_wrench': LaunchConfiguration('cartesian_admittance_require_wrench'),
                    'wrench_bias_duration':
                        LaunchConfiguration('cartesian_admittance_wrench_bias_duration'),
                    'wrench_filter_alpha':
                        LaunchConfiguration('cartesian_admittance_wrench_filter_alpha'),
                    'max_linear_velocity':
                        LaunchConfiguration('cartesian_admittance_max_linear_velocity'),
                    'max_angular_velocity':
                        LaunchConfiguration('cartesian_admittance_max_angular_velocity'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
                output='screen',
            )
        )

    return TimerAction(
        period=LaunchConfiguration('jparse_startup_delay'),
        actions=actions,
        condition=IfCondition(LaunchConfiguration('launch_cartesian_admittance')),
    )


def load_robot_profile(profile_file, profile_name, robot_name):
    selected_profile = profile_name.strip()
    if selected_profile in ('', 'auto'):
        selected_profile = robot_name

    if not os.path.exists(profile_file):
        print(f"[mur_620.launch] Robot profile file does not exist: {profile_file}")
        return selected_profile, {}

    with open(profile_file, 'r', encoding='utf-8') as config:
        profiles = yaml.safe_load(config) or {}

    profile = profiles.get('robots', {}).get(selected_profile, {})
    if not profile:
        print(
            f"[mur_620.launch] Robot profile '{selected_profile}' not found in {profile_file}; "
            "using generic fallback values."
        )
    return selected_profile, profile


def resolve_profile_path(package_share, path):
    if not path:
        return path
    if os.path.isabs(path):
        return path
    return os.path.join(package_share, path)


def as_launch_bool(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value).lower() if str(value).lower() in ('true', 'false') else str(value)


def load_integrated_controller_config():
    helper_path = os.path.join(
        get_package_share_directory('mur_launch_hardware'),
        'launch',
        'integrated_controller_config.py',
    )
    spec = importlib.util.spec_from_file_location(
        'mur_launch_hardware_integrated_controller_config',
        helper_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    ur_type = LaunchConfiguration('ur_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    mur_launch_hardware = get_package_share_directory('mur_launch_hardware')
    mur_description_path = get_package_share_directory('mur_description')
    ur_robot_driver_path = get_package_share_directory('ur_robot_driver')
    xacro_file = os.path.join(mur_description_path, 'urdf', 'mur_620.gazebo.xacro')
    controllers_file = LaunchConfiguration('controllers_file').perform(context)
    update_rate_config_file = os.path.join(
        ur_robot_driver_path,
        'config',
        f'{ur_type}_update_rate.yaml',
    )
    profile_name, robot_profile = load_robot_profile(
        LaunchConfiguration('robot_profile_file').perform(context),
        LaunchConfiguration('robot_profile').perform(context),
        robot_name,
    )
    arm_profiles = robot_profile.get('arms', {})

    def profile_value(arg_name, profile_key, fallback):
        launch_value = LaunchConfiguration(arg_name).perform(context).strip()
        if launch_value:
            return launch_value
        if profile_key in robot_profile:
            return as_launch_bool(robot_profile[profile_key])
        return fallback

    def arm_profile_value(side, arg_name, profile_key, fallback, *, path=False):
        launch_value = LaunchConfiguration(arg_name).perform(context).strip()
        if launch_value:
            return launch_value
        value = arm_profiles.get(side, {}).get(profile_key, fallback)
        if path:
            return resolve_profile_path(mur_launch_hardware, value)
        return str(value)

    use_lift = profile_value('use_lift', 'use_lift', 'true')
    launch_lift_l = profile_value('launch_lift_l', 'use_lift', use_lift)
    launch_lift_r = profile_value('launch_lift_r', 'use_lift', use_lift)
    battery_node_id = profile_value('battery_node_id', 'battery_node_id', '0x0240')
    ur_l_xyz = arm_profile_value('l', 'ur_l_xyz', 'mount_xyz', '0.0 0.0 0.0')
    ur_l_rpy = arm_profile_value('l', 'ur_l_rpy', 'mount_rpy', '0.0 0.0 0.0')
    ur_r_xyz = arm_profile_value('r', 'ur_r_xyz', 'mount_xyz', '0.0 0.0 0.0')
    ur_r_rpy = arm_profile_value('r', 'ur_r_rpy', 'mount_rpy', '0.0 0.0 3.14159265359')
    home_custom_l_shoulder_pan = str(
        arm_profiles.get('l', {}).get('home_custom_shoulder_pan', '0.0')
    )
    home_custom_r_shoulder_pan = str(
        arm_profiles.get('r', {}).get('home_custom_shoulder_pan', '0.0')
    )
    kinematics_params_file_l = arm_profile_value(
        'l',
        'kinematics_params_file_l',
        'kinematics_params_file',
        os.path.join(mur_launch_hardware, 'calibration', 'calibration_UR10_18.yaml'),
        path=True,
    )
    kinematics_params_file_r = arm_profile_value(
        'r',
        'kinematics_params_file_r',
        'kinematics_params_file',
        os.path.join(mur_launch_hardware, 'calibration', 'calibration_UR10_12.yaml'),
        path=True,
    )

    print(
        f"[mur_620.launch] robot_name={robot_name}, robot_profile={profile_name}, "
        f"use_lift={use_lift}, launch_lift_l={launch_lift_l}, "
        f"launch_lift_r={launch_lift_r}, battery_node_id={battery_node_id}"
    )
    print(
        f"[mur_620.launch] UR10_l kinematics={kinematics_params_file_l}, "
        f"mount_xyz='{ur_l_xyz}', mount_rpy='{ur_l_rpy}'"
    )
    print(
        f"[mur_620.launch] UR10_r kinematics={kinematics_params_file_r}, "
        f"mount_xyz='{ur_r_xyz}', mount_rpy='{ur_r_rpy}'"
    )

    integrated_config = load_integrated_controller_config().make_integrated_controller_params(
        context,
        robot_name=robot_name,
        ur_l_xyz=ur_l_xyz,
        ur_l_rpy=ur_l_rpy,
        ur_r_xyz=ur_r_xyz,
        ur_r_rpy=ur_r_rpy,
    )
    integrated_inputs = integrated_config['inputs']
    integrated_params_by_side = integrated_config['params_by_side']
    ur_mount_rpy = integrated_config['ur_mount_rpy']
    ur_collision_base_xyz = integrated_config['ur_collision_base_xyz']
    print(
        "[mur_620.launch] integrated admittance: "
        f"use_ft={integrated_inputs['use_ft_sensor']}, "
        f"reset_equilibrium_on_zero={integrated_inputs['reset_equilibrium_on_zero']}, "
        f"wrench_in_tcp_frame={LaunchConfiguration('integrated_controller_wrench_in_tcp_frame').perform(context)}, "
        f"admittance={integrated_inputs['admittance']}, "
        f"pose_error_gain={integrated_inputs['pose_error_gain']}, "
        f"wrench_twist_gain={integrated_inputs['wrench_twist_gain']}, "
        f"wrench_sign={integrated_inputs['wrench_sign']}"
    )
    print(
        "[mur_620.launch] integrated collision base transforms: "
        f"UR10_l xyz={ur_collision_base_xyz['l']} rpy={ur_mount_rpy['l']}, "
        f"UR10_r xyz={ur_collision_base_xyz['r']} rpy={ur_mount_rpy['r']}"
    )

    visual_mesh_flags = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            'use_base_visual_mesh',
            'use_top_visual_mesh',
            'use_wheel_visual_mesh',
            'use_caster_visual_mesh',
            'use_lift_visual_mesh',
            'use_laser_visual_mesh',
        )
    }

    doc = xacro.process_file(xacro_file, mappings={
        'robot_namespace': robot_name,
        'tf_prefix': robot_name,
        'tf_prefix_mir': robot_name,
        'use_arms': LaunchConfiguration('use_arms').perform(context),
        'use_camera': LaunchConfiguration('use_camera').perform(context),
        'use_lidar': LaunchConfiguration('use_lidar').perform(context),
        'use_lift': use_lift,
        'use_simple_collisions': LaunchConfiguration('use_simple_collisions').perform(context),
        'use_simple_visuals': LaunchConfiguration('use_simple_visuals').perform(context),
        'use_high_quality_visuals': LaunchConfiguration('use_high_quality_visuals').perform(context),
        'ur_type': ur_type,
        'kinematics_params_l': kinematics_params_file_l,
        'kinematics_params_r': kinematics_params_file_r,
        'ur_l_xyz': ur_l_xyz,
        'ur_l_rpy': ur_l_rpy,
        'ur_r_xyz': ur_r_xyz,
        'ur_r_rpy': ur_r_rpy,
        **visual_mesh_flags,
    })

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{robot_name}_rsp',
        namespace=robot_name,
        condition=IfCondition(LaunchConfiguration('publish_mur_tf')),
        parameters=[
            {'robot_description': doc.toxml()},
            {'use_sim_time': use_sim_time},
            {'frame_prefix': f'{robot_name}/'},
        ],
        remappings=[('joint_states', '/joint_states')],
        output='screen',
    )

    return [
        robot_state_publisher,
        make_bms_can_node(robot_name, battery_node_id),
        make_mir_hardware_launch(robot_name, use_sim_time),
        make_fake_mir_wheel_joint_publisher(robot_name),
        make_arm_velocity_safety_node(robot_name),
        make_moveit_controller_proxies(robot_name),
        make_jparse_nodes(robot_name),
        make_integrated_cartesian_move_action_nodes(robot_name),
        make_cartesian_admittance_nodes(robot_name),
        make_moveit_launch(
            robot_name,
            home_custom_l_shoulder_pan,
            home_custom_r_shoulder_pan,
        ),
        make_lift_driver('l', robot_name, launch_lift_l),
        make_lift_driver('r', robot_name, launch_lift_r),
        make_lift_joint_state_bridge(robot_name, use_lift),
        make_ur_driver(
            'l',
            robot_name,
            make_arm_controllers_file(
                controllers_file,
                robot_name,
                'UR10_l',
                integrated_params_by_side['l'],
            ),
            update_rate_config_file,
            kinematics_params_file_l,
        ),
        make_ur_driver(
            'r',
            robot_name,
            make_arm_controllers_file(
                controllers_file,
                robot_name,
                'UR10_r',
                integrated_params_by_side['r'],
            ),
            update_rate_config_file,
            kinematics_params_file_r,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(declare_arguments() + [OpaqueFunction(function=launch_setup)])
