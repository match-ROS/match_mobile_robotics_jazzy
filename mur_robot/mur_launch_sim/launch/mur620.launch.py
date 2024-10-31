from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
import os
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command, IfElseSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, OpaqueFunction, TimerAction
from launch.event_handlers import OnProcessExit
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path
from launch_ros.actions import SetParameter



def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    description_file = LaunchConfiguration("description_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    moveit_launch_file = LaunchConfiguration("moveit_launch_file")

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items(),
    )

    nodes_to_launch = [
        ur_moveit_launch,
    ]

    return nodes_to_launch


def generate_launch_description():
    mir_launch_dir = get_package_share_directory('mur_launch_sim')
    mur_base_launch_path = os.path.join(mir_launch_dir, 'launch', 'mur_base.launch.py')
    SetParameter(name='use_sim_time', value=True),  # Set globally for all nodes

    #################### UR section ####################

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur10e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "config", "ur_controllers.yaml"]
            ),
            description="Absolute path to YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_launch_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("mur_moveit_config"),
                    "launch",
                    "ur_moveit.launch.py",
                ]
            ),
            description="Absolute path for MoveIt launch file, part of a config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )

    load_forward_position_controller_l = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'forward_position_controller_l'],
        output='screen'
    )

    load_forward_position_controller_r = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'forward_position_controller_r'],
        output='screen'
    )

    load_forward_velocity_controller_l = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'forward_velocity_controller_l'],
        output='screen'
    )

    load_forward_velocity_controller_r = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'forward_velocity_controller_r'],
        output='screen'
    )

    load_joint_trajectory_controller_l = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller_l'],
        output='screen'
    )

    load_joint_trajectory_controller_r = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller_r'],
        output='screen'
    )

    # robot_description_file_path = os.path.join(get_package_share_directory("mur_desciption"), "urdf", "mur_620.gazebo.xacro")
    moveit_config = (
        MoveItConfigsBuilder(robot_name="mur620", package_name="mur_moveit_config")
        .robot_description_semantic(Path("srdf") / "mur620.srdf.xacro", {"prefix": "mur620","model_name": "mur620"})
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    moveit_move_to_home_node = Node(
            package='mur_launch_sim',
            executable='moveit_move_to_home.py',
            name='moveit_move_to_home_node',
            output='screen',
                    parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            moveit_config.moveit_cpp,
            {'use_sim_time': True},
        ],
        )


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mur_base_launch_path)
        ),
        load_forward_position_controller_l,
        load_forward_position_controller_r,
        load_forward_velocity_controller_l,
        load_forward_velocity_controller_r,
        load_joint_trajectory_controller_l,
        load_joint_trajectory_controller_r,
        # wait until all controllers are loaded
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller_r,
                on_exit=[moveit_move_to_home_node],
            )
        ),
        *declared_arguments,
        OpaqueFunction(function=launch_setup)

    ])