import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    world = LaunchConfiguration('world').perform(context)

    mur_description_path = get_package_share_directory('mur_desciption')
    mir_gazebo_path = get_package_share_directory('mir_gazebo')
    gz_sim_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    xacro_path = os.path.join(mur_description_path, 'urdf', 'mur_620.gazebo.xacro')

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': f'{world}.world -v 4 -r'
        }.items()
    )

    def spawn_robot(robot_name, x, y):
        doc = xacro.process_file(xacro_path, mappings={
            'use_sim': 'true',
            'tf_prefix': robot_name,
            'robot_namespace': robot_name
        })
        robot_desc = doc.toxml()

        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_rsp',
            namespace=robot_name,
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': use_sim_time, 'frame_prefix': f'{robot_name}/'}
            ],
            output='screen'
        )

        spawn_node = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-string', robot_desc,
                '-name', robot_name,
                '-x', str(x), '-y', str(y), '-z', '0.07'
            ],
            output='screen',
            emulate_tty=True
        )

        return [spawn_node, rsp_node]

    nodes = []
    nodes += spawn_robot('mur620a', 0.0, 0.0)
    nodes += spawn_robot('mur620b', 1.5, 0.0)

    return [gazebo] + nodes

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