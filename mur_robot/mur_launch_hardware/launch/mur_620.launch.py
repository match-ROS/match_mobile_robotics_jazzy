"""Generic hardware launch for a MUR620 with two UR arms."""

import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def declare_arguments():
    mur_launch_hardware = get_package_share_directory('mur_launch_hardware')

    return [
        DeclareLaunchArgument('robot_name', default_value='mur620'),
        DeclareLaunchArgument('ur_type', default_value='ur10'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_mur_tf', default_value='true'),
        DeclareLaunchArgument('use_arms', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_lift', default_value='true'),
        DeclareLaunchArgument('use_simple_collisions', default_value='false'),
        DeclareLaunchArgument('use_simple_visuals', default_value='false'),
        DeclareLaunchArgument('use_high_quality_visuals', default_value='false'),
        DeclareLaunchArgument('use_base_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_top_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_wheel_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_caster_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_lift_visual_mesh', default_value='false'),
        DeclareLaunchArgument('use_laser_visual_mesh', default_value='false'),
        DeclareLaunchArgument(
            'ur_l_xyz',
            default_value='0.0 0.0 0.0',
        ),
        DeclareLaunchArgument('ur_l_rpy', default_value='0.0 0.0 0.0'),
        DeclareLaunchArgument(
            'ur_r_xyz',
            default_value='0.0 0.0 0.0',
        ),
        DeclareLaunchArgument('ur_r_rpy', default_value='0.0 0.0 3.14159265359'),
        DeclareLaunchArgument('launch_ur_l', default_value='true'),
        DeclareLaunchArgument('launch_ur_r', default_value='true'),
        DeclareLaunchArgument('robot_ip_l', default_value='UR10_l'),
        DeclareLaunchArgument('robot_ip_r', default_value='UR10_r'),
        DeclareLaunchArgument('reverse_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('headless_mode', default_value='false'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false'),
        DeclareLaunchArgument('launch_dashboard_client', default_value='true'),
        DeclareLaunchArgument('initial_joint_controller', default_value='scaled_joint_trajectory_controller'),
        DeclareLaunchArgument('activate_joint_controller', default_value='true'),
        DeclareLaunchArgument('controller_spawner_timeout', default_value='10'),
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
            default_value=os.path.join(
                mur_launch_hardware,
                'calibration',
                'calibration_UR10_18.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'kinematics_params_file_r',
            default_value=os.path.join(
                mur_launch_hardware,
                'calibration',
                'calibration_UR10_12.yaml',
            ),
        ),
    ]


def ur_description_arguments(side):
    return {
        'ur_type': LaunchConfiguration('ur_type'),
        'robot_ip': LaunchConfiguration(f'robot_ip_{side}'),
        'safety_limits': LaunchConfiguration('safety_limits'),
        'safety_pos_margin': LaunchConfiguration('safety_pos_margin'),
        'safety_k_position': LaunchConfiguration('safety_k_position'),
        'kinematics_params_file': LaunchConfiguration(f'kinematics_params_file_{side}'),
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


def make_arm_controllers_file(source_file, robot_name, arm_name):
    with open(source_file, 'r', encoding='utf-8') as config:
        contents = config.read()

    contents = contents.replace('$(var tf_prefix)', f'{arm_name}/')

    out_dir = os.path.join(tempfile.gettempdir(), 'mur_launch_hardware')
    os.makedirs(out_dir, exist_ok=True)
    out_file = os.path.join(out_dir, f'{robot_name}_{arm_name}_ur_controllers.yaml')

    with open(out_file, 'w', encoding='utf-8') as config:
        config.write(contents)

    return out_file


def make_ur_driver(side, robot_name, controllers_file, update_rate_config_file):
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
            ]},
        ],
    )

    trajectory_until_node = Node(
        package='ur_robot_driver',
        executable='trajectory_until_node',
        name='trajectory_until_node',
        namespace=namespace,
        output='screen',
        parameters=[{'motion_controller': LaunchConfiguration('initial_joint_controller')}],
    )

    ur_rsp = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst=f'/{namespace}/unused_tf'),
        SetRemap(src='/tf_static', dst=f'/{namespace}/unused_tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_rsp_launch),
            launch_arguments=ur_description_arguments(side).items(),
        ),
    ])

    def add_selected_controller(context):
        selected = selected_controller(context)
        active = list(controllers_active)
        inactive = list(controllers_inactive)
        if activate_joint_controller.perform(context) == 'true':
            active.append(selected)
            if selected in inactive:
                inactive.remove(selected)
        if use_mock_hardware.perform(context) == 'true' and 'tcp_pose_broadcaster' in active:
            active.remove('tcp_pose_broadcaster')

        return [
            controller_spawner(namespace, active),
            controller_spawner(namespace, inactive, active=False),
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


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    ur_type = LaunchConfiguration('ur_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    mur_description_path = get_package_share_directory('mur_description')
    ur_robot_driver_path = get_package_share_directory('ur_robot_driver')
    xacro_file = os.path.join(mur_description_path, 'urdf', 'mur_620.gazebo.xacro')
    controllers_file = LaunchConfiguration('controllers_file').perform(context)
    update_rate_config_file = os.path.join(
        ur_robot_driver_path,
        'config',
        f'{ur_type}_update_rate.yaml',
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
        'use_lift': LaunchConfiguration('use_lift').perform(context),
        'use_simple_collisions': LaunchConfiguration('use_simple_collisions').perform(context),
        'use_simple_visuals': LaunchConfiguration('use_simple_visuals').perform(context),
        'use_high_quality_visuals': LaunchConfiguration('use_high_quality_visuals').perform(context),
        'ur_type': ur_type,
        'ur_l_xyz': LaunchConfiguration('ur_l_xyz').perform(context),
        'ur_l_rpy': LaunchConfiguration('ur_l_rpy').perform(context),
        'ur_r_xyz': LaunchConfiguration('ur_r_xyz').perform(context),
        'ur_r_rpy': LaunchConfiguration('ur_r_rpy').perform(context),
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
        make_ur_driver(
            'l',
            robot_name,
            make_arm_controllers_file(controllers_file, robot_name, 'UR10_l'),
            update_rate_config_file,
        ),
        make_ur_driver(
            'r',
            robot_name,
            make_arm_controllers_file(controllers_file, robot_name, 'UR10_r'),
            update_rate_config_file,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(declare_arguments() + [OpaqueFunction(function=launch_setup)])
