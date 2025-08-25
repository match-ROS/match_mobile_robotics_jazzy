import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    world = LaunchConfiguration('world').perform(context)

    # Pfade
    mur_description_path = get_package_share_directory('mur_description')
    mir_gazebo_path = get_package_share_directory('mir_gazebo')
    gz_sim_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    xacro_path = os.path.join(mur_description_path, 'urdf', 'mur_620.gazebo.xacro')

    # Starte Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': f'{world}.world -v 4 -r'}.items()
        #launch_arguments={'gz_args': f'{world}.world -r -s'}.items()
    )

    # Bridge für /clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=['/mur620a/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        remappings=[
            ('/mur620a/scan', '/mur620a/scan')  # optional
        ],
        output='screen'
    )

    def spawn_robot(robot_name, x, y):
        # Generiere URDF
        doc = xacro.process_file(xacro_path, mappings={
            'use_sim': 'true',
            'tf_prefix': robot_name,
            'robot_namespace': robot_name
        })
        robot_desc = doc.toxml()

        # State Publisher im Namespace
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_rsp',
            namespace=robot_name,
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': use_sim_time == 'true'},
                {'frame_prefix': f'{robot_name}/'} 
            ],
            output='screen'
        )

        # zusätzlich globaler Publisher (damit gz_ros_control globales /robot_description findet)
        global_rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_global_rsp',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': use_sim_time == 'true'}
            ],
            output='screen'
        )

        # Controller Manager im Namespace mit Remap
        control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=robot_name,
            name='controller_manager',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': use_sim_time == 'true'}
            ],
            remappings=[
                ('/robot_description', f'/{robot_name}/robot_description')
            ],
            output='screen'
        )

        # Roboter in Gazebo einfügen
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'{robot_name}_spawn',
            arguments=[
                '-string', robot_desc,
                '-name', robot_name,
                '-x', str(x), '-y', str(y), '-z', '0.07'
            ],
            output='screen'
        )

        return [spawn_node, rsp_node, global_rsp_node, control_node]

    # Roboter erzeugen
    nodes = []
    nodes += spawn_robot('mur620a', 0.0, 0.0)
    nodes += spawn_robot('mur620b', 1.5, 0.0)
    nodes += spawn_robot('mur620c', 3.5, 0.0)
    nodes += spawn_robot('mur620d', 5.5, 0.0)

    return [gazebo, clock_bridge,lidar_bridge] + nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='maze'),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(get_package_share_directory('mir_gazebo'), 'worlds')
        ),
        OpaqueFunction(function=launch_setup)
    ])
