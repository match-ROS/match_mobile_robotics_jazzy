import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, OpaqueFunction, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command, IfElseSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

  
def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    mir_gazebo_path = os.path.join(get_package_share_directory('mir_gazebo'))
    
    mur_description_path = os.path.join(
        get_package_share_directory('mur_desciption'))
   
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(mir_gazebo_path, 'worlds')
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
    
    xacro_file = os.path.join(mur_description_path, 'urdf', 'mur_620.gazebo.xacro')

    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}]
    )

    gz_robot_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.07',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'mur_620',
                   '-allow_renaming', 'false'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_mobile_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mobile_base_controller'],
        output='screen'
    )

    load_right_lift_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'lift_controller_r'],
        output='screen'
    )

    load_left_lift_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'lift_controller_l'],
        output='screen'
    )

    # launch rqt_robot_steering
    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        output='screen'
    )

    #launch rqt_controller_manager
    rqt_controller_manager = Node(
        package='rqt_controller_manager',
        executable='rqt_controller_manager',
        output='screen'
    )

    # launch repub_twist.py
    repub_twist = Node(
        package='mir_gazebo',
        executable='repub_twist.py',
        output='screen'
    )

    # Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/b_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan','/f_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    rviz_config_file = os.path.join(mir_gazebo_path, 'rviz', 'mir600.rviz')

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, {'use_sim_time': True}],
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_robot_spawner,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_mobile_base_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_mobile_base_controller,
                on_exit=[load_right_lift_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_right_lift_controller,
                on_exit=[load_left_lift_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_left_lift_controller,
                on_exit=[rqt_controller_manager],
            )
        ),
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        gz_robot_spawner,
        ros_gz_bridge,
        rviz,
        rqt_robot_steering,
        repub_twist,
        # *declared_arguments,
        # OpaqueFunction(function=launch_setup)
    ])


