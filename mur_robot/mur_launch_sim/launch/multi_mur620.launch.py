from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, OpaqueFunction, TimerAction
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Define namespace arguments for both robots
    declared_arguments = [
        DeclareLaunchArgument("robot1_ns", default_value="robot1", description="Namespace for robot 1"),
        DeclareLaunchArgument("robot2_ns", default_value="robot2", description="Namespace for robot 2"),
        DeclareLaunchArgument("tf_prefix1", default_value="robot1_", description="TF prefix for robot 1"),
        DeclareLaunchArgument("tf_prefix2", default_value="robot2_", description="TF prefix for robot 2"),
        DeclareLaunchArgument("world", default_value="maze", description="Gz sim World"),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    mir_gazebo_path = os.path.join(get_package_share_directory('mir_gazebo'))

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(mir_gazebo_path, 'worlds')
            ]
        )

    # Get the path to the existing launch file
    launch_file_path = os.path.join(
        FindPackageShare("mur_launch_sim").find("mur_launch_sim"),
        "launch",
        "mur620.launch.py",
    )

    # Group action for robot 1 with namespace and tf_prefix
    robot1 = GroupAction([
        PushRosNamespace(LaunchConfiguration("robot1_ns")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments={
                "tf_prefix": LaunchConfiguration("tf_prefix1"),
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
                "tf_prefix": LaunchConfiguration("tf_prefix2"),
                # add other specific arguments as needed
            }.items(),
        ),
    ])

    print("world: ", LaunchConfiguration('world'))

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

    #return LaunchDescription(declared_arguments + [robot1, robot2, gazebo])
    return LaunchDescription(declared_arguments + [gazebo_resource_path,
                                                   gazebo,
                                                   # wait until all controllers are loaded
                                                    RegisterEventHandler(
                                                         event_handler=OnProcessExit(
                                                              target_action=gazebo,
                                                              on_exit=[robot1, robot2],
                                                         )
                                                    )])

    return LaunchDescription([
        gazebo_resource_path,
        gazebo,
        # wait until all controllers are loaded
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=gazebo,
        #         on_exit=[robot1, robot2],
        #     )
        # ),
        *declared_arguments,
    ])