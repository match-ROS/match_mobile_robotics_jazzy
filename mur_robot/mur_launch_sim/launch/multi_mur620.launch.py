from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Define namespace arguments for both robots
    declared_arguments = [
        DeclareLaunchArgument("robot1_ns", default_value="robot1", description="Namespace for robot 1"),
        DeclareLaunchArgument("robot2_ns", default_value="robot2", description="Namespace for robot 2"),
        DeclareLaunchArgument("tf_prefix1", default_value="", description="TF prefix for robot 1"),
        DeclareLaunchArgument("tf_prefix2", default_value="robot2_", description="TF prefix for robot 2"),
        DeclareLaunchArgument("world", default_value="maze", description="Gz sim World"),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    mir_gazebo_path = os.path.join(get_package_share_directory('mir_gazebo'))

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(mir_gazebo_path, 'worlds')
    )

    # Get the path to the existing launch file
    launch_file_path = os.path.join(
        FindPackageShare("mur_launch_sim").find("mur_launch_sim"),
        "launch",
        "mur620_slim.launch.py",
    )

    # Group action for robot 1 with namespace and tf_prefix
    robot1 = GroupAction([
        PushRosNamespace(LaunchConfiguration("robot1_ns")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments={
                #"tf_prefix": LaunchConfiguration("tf_prefix1"),
                # add other specific arguments as needed
            }.items(),
        ),
    ])

    # Group action for robot 2 with different namespace and tf_prefix
    robot2 = GroupAction([
        PushRosNamespace(LaunchConfiguration("robot2_ns")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments={
                #"tf_prefix": LaunchConfiguration("tf_prefix2"),
                # add other specific arguments as needed
            }.items(),
        ),
    ])

    # Launch Gazebo first
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'),
                '.world',
                ' -v 4',
                ' -r'
            ])
        ]
    )

    # TimerAction to delay launching of robots by 5 seconds
    start_robots = TimerAction(
        period=5.0,
        actions=[robot1]
    )

    # Define the launch description
    return LaunchDescription(declared_arguments + [gazebo_resource_path, gazebo, start_robots])
