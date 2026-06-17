from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_REST_AUTH = (
    'Basic '
    'ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='
)


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mir_hostname = LaunchConfiguration('mir_hostname')
    mir_port = LaunchConfiguration('mir_port')
    mir_restapi_auth = LaunchConfiguration('mir_restapi_auth')

    mir_driver_launch = PathJoinSubstitution([
        FindPackageShare('mir_driver'),
        'launch',
        'mir_headless_launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Namespace to push all MiR ROS 2 interfaces into.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mir_hostname', default_value='192.168.12.20'),
        DeclareLaunchArgument('mir_port', default_value='9090'),
        DeclareLaunchArgument('mir_type', default_value='mir_600', description='Documented hardware variant: mir_100, mir_200, or mir_600.'),
        DeclareLaunchArgument('mir_restapi_auth', default_value=DEFAULT_REST_AUTH),
        DeclareLaunchArgument('robot_state_publisher_enabled', default_value='false'),
        DeclareLaunchArgument('bridge_light_commands', default_value='true'),
        DeclareLaunchArgument('set_default_light_on_start', default_value='false'),
        DeclareLaunchArgument('localization_type', default_value='robot_pose'),
        DeclareLaunchArgument('external_localization', default_value='false'),
        DeclareLaunchArgument('run_rosapi_audit', default_value='false'),
        DeclareLaunchArgument('localization_topic', default_value='/qualisys_map/mur620a/pose'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mir_driver_launch),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'mir_hostname': mir_hostname,
                'mir_port': mir_port,
                'robot_state_publisher_enabled': LaunchConfiguration('robot_state_publisher_enabled'),
            }.items(),
        ),

        Node(
            package='mir_restapi',
            executable='mir_restapi_server',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'mir_hostname': mir_hostname,
                'mir_restapi_auth': mir_restapi_auth,
            }],
            output='screen',
        ),
        Node(
            package='mir_launch_hardware',
            executable='mir_battery_state_publisher',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'mir_hostname': mir_hostname,
                'mir_restapi_auth': mir_restapi_auth,
            }],
            output='screen',
        ),
        Node(
            package='mir_launch_hardware',
            executable='rgb_control',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'mir_hostname': mir_hostname,
                'mir_port': mir_port,
                'bridge_light_commands': LaunchConfiguration('bridge_light_commands'),
                'set_default_on_start': LaunchConfiguration('set_default_light_on_start'),
            }],
            output='screen',
        ),
        Node(
            package='mir_launch_hardware',
            executable='mir_pose_simple',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'localization_type': LaunchConfiguration('localization_type'),
                'mocap_topic': LaunchConfiguration('localization_topic'),
                'robot_pose_topic': 'robot_pose',
                'odom_topic': 'odom',
            }],
            output='screen',
        ),
        Node(
            package='mir_launch_hardware',
            executable='mir_rosapi_audit',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'mir_hostname': mir_hostname,
                'mir_port': mir_port,
            }],
            condition=IfCondition(LaunchConfiguration('run_rosapi_audit')),
            output='screen',
        ),
        Node(
            package='mir_launch_hardware',
            executable='external_localization_broadcaster',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'tf_prefix': namespace,
                'localization_topic': LaunchConfiguration('localization_topic'),
            }],
            condition=IfCondition(LaunchConfiguration('external_localization')),
            output='screen',
        ),
    ])
