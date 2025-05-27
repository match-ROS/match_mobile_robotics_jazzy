from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Declare robot arguments
    declared_arguments = [
        DeclareLaunchArgument('mur_ns', default_value='mur620'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),
        DeclareLaunchArgument('prefix', default_value='UR10'),
        DeclareLaunchArgument('tool', default_value='schunk_emh_rp_045'),
        DeclareLaunchArgument('lift', default_value='true'),
        DeclareLaunchArgument('simulate_camera', default_value='false'),
        DeclareLaunchArgument('enable_dual_collison_avoidance', default_value='true'),
        DeclareLaunchArgument('robot_model_name', default_value='mur620')
    ]

    # Paths
    mur_desc_path = get_package_share_directory('mur_desciption')
    mir_gazebo_path = get_package_share_directory('mir_gazebo')
    urdf_file = os.path.join(mur_desc_path, 'urdf', 'mur_620.gazebo.xacro')

    # Resolve xacro
    xacro_args = {
        'use_sim': 'true',
        'tool': LaunchConfiguration('tool'),
        'use_lift': LaunchConfiguration('lift'),
        'simulate_camera': LaunchConfiguration('simulate_camera')
    }
    doc = xacro.process_file(urdf_file, mappings={k: v.perform({}) for k, v in xacro_args.items()})
    robot_desc = doc.toprettyxml(indent='  ')
    robot_description = {'robot_description': robot_desc}

    use_sim_time = {'use_sim_time': True}

    # Robot state publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=LaunchConfiguration('mur_ns'),
        parameters=[robot_description, use_sim_time]
    )

    # Spawn robot in Gazebo
    gz_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=LaunchConfiguration('mur_ns'),
        arguments=['-string', robot_desc,
                   '-x', LaunchConfiguration('robot_x'),
                   '-y', LaunchConfiguration('robot_y'),
                   '-z', '0.05',
                   '-Y', LaunchConfiguration('robot_yaw'),
                   '-name', LaunchConfiguration('mur_ns')],
    )

    # Load controllers
    load_jsc = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_mobile_base = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mobile_base_controller'],
        output='screen'
    )

    load_lift_left = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'lift_controller_l'],
        output='screen'
    )

    load_lift_right = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'lift_controller_r'],
        output='screen'
    )
    repub_twist = Node(
        package='mir_gazebo',
        executable='repub_twist.py',
        output='screen'
    )



    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=LaunchConfiguration('mur_ns'),
        output='log',
        arguments=['-d', os.path.join(mir_gazebo_path, 'rviz', 'mir600.rviz')],
        parameters=[use_sim_time]
    )

    # Joint state bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/b_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/f_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',  # <--- Das ist neu!
            '/model/UR10_l/link/tool0/ft_sensor_l@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            '/ft_sensor_r@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'
        ],
        output='screen'
    )

    return LaunchDescription(
        declared_arguments + [
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(mir_gazebo_path, 'worlds')),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gz_spawner,
                    on_exit=[load_jsc]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_jsc,
                    on_exit=[load_mobile_base]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_mobile_base,
                    on_exit=[load_lift_left]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_lift_left,
                    on_exit=[load_lift_right]
                )
            ),
            rsp_node,
            gz_spawner,
            gz_bridge,
            rviz
        ]
    )
