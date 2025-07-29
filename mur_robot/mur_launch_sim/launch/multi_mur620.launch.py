from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("world", default_value="maze", description="Gazebo world"),
    ]
        # Path setups
    mur_launch_sim_path = get_package_share_directory("mur_launch_sim")
    
    mir_gazebo_path = get_package_share_directory('mir_gazebo')

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(mir_gazebo_path, 'worlds')
            ]
        )

    # arguments = LaunchDescription([
    #             DeclareLaunchArgument('world', default_value='maze',
    #                       description='Gz sim World'),
    #        ]
    # )

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

    # Robots definition
    robots = [
        {"name": "mur620a", "x": "-2.3", "y": "-1.0", "yaw": "0.0", "mur_ns": "mur620a"}
    ]

    # robots = [
    #     {"name": "mur620a", "x": "-2.3", "y": "-1.0", "yaw": "0.0", "mur_ns": "mur620a"},
    #     {"name": "mur620b", "x": "-2.3", "y": "1.0", "yaw": "0.0", "mur_ns": "mur620b"},
    #     {"name": "mur620c", "x": "2.3", "y": "-1.0", "yaw": "3.14159", "mur_ns": "mur620c"},
    #     {"name": "mur620d", "x": "2.3", "y": "1.0", "yaw": "3.14159", "mur_ns": "mur620d"},
    # ]

    robot_groups = []
    for robot in robots:
        group = GroupAction([
            #PushRosNamespace(robot["name"]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mur_launch_sim_path, "launch", "mur620_slim.launch.py")
                ),
                launch_arguments={
                    "robot_x": robot["x"],
                    "robot_y": robot["y"],
                    "robot_yaw": robot["yaw"],
                    "tf_prefix": robot["name"],
                    "mur_ns": robot["mur_ns"]
                }.items()
            )
        ])
        robot_groups.append(group)

    return LaunchDescription(
        declared_arguments + [
            gazebo_resource_path,
            gazebo,
            #TimerAction(period=5.0, actions=robot_groups)
            *robot_groups,
        ]
    )
