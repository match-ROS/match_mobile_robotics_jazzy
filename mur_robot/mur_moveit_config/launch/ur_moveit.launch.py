# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Felix Exner
import os
import yaml

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def arm_controller_config(controller_namespace):
    if controller_namespace:
        controller_prefix = f"/{controller_namespace}"
    else:
        controller_prefix = ""

    left_controller = f"{controller_prefix}/moveit_joint_trajectory_controller_l"
    right_controller = f"{controller_prefix}/moveit_joint_trajectory_controller_r"
    left_lift_controller = (
        f"{controller_prefix}/moveit_joint_trajectory_controller_lift_l"
    )
    right_lift_controller = (
        f"{controller_prefix}/moveit_joint_trajectory_controller_lift_r"
    )

    left_joints = [
        "UR10_l/shoulder_pan_joint",
        "UR10_l/shoulder_lift_joint",
        "UR10_l/elbow_joint",
        "UR10_l/wrist_1_joint",
        "UR10_l/wrist_2_joint",
        "UR10_l/wrist_3_joint",
    ]
    right_joints = [
        "UR10_r/shoulder_pan_joint",
        "UR10_r/shoulder_lift_joint",
        "UR10_r/elbow_joint",
        "UR10_r/wrist_1_joint",
        "UR10_r/wrist_2_joint",
        "UR10_r/wrist_3_joint",
    ]
    left_lift_joints = ["left_lift_joint"] + left_joints
    right_lift_joints = ["right_lift_joint"] + right_joints

    return {
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 1.2,
            "allowed_goal_duration_margin": 0.5,
            "allowed_start_tolerance": 0.01,
            "execution_duration_monitoring": False,
        },
        "moveit_simple_controller_manager": {
            "controller_names": [
                left_controller,
                right_controller,
                left_lift_controller,
                right_lift_controller,
            ],
            left_controller: {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": left_joints,
            },
            right_controller: {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": right_joints,
            },
            left_lift_controller: {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": left_lift_joints,
            },
            right_lift_controller: {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": right_lift_joints,
            },
        },
    }


def robot_description_source(robot_name):
    mur_description_path = get_package_share_directory("mur_description")
    xacro_file = os.path.join(mur_description_path, "urdf", "mur_620.gazebo.xacro")
    controllers_yaml = os.path.join(
        "/tmp",
        "mur_launch_sim",
        f"{robot_name.replace('/', '_')}_mur_controllers.yaml",
    )
    if not os.path.exists(controllers_yaml):
        controllers_yaml = os.path.join(
            get_package_share_directory("mir_description"),
            "config",
            "mur_controllers.yaml",
        )

    mappings = {
        "use_sim": "true",
        "tf_prefix": robot_name,
        "tf_prefix_mir": robot_name,
        "robot_namespace": robot_name,
        "simulation_controllers": controllers_yaml,
    }
    return xacro_file, mappings


def declare_arguments():
    default_rviz_config = os.path.join(
        get_package_share_directory("mur_moveit_config"),
        "config",
        "mur620a.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz", default_value="false", description="Launch RViz?"
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file",
            ),
            DeclareLaunchArgument(
                "rviz_delay",
                default_value="10.0",
                description="Seconds to wait before launching RViz after MoveIt startup",
            ),
            DeclareLaunchArgument(
                "ur_type",
                description="Typo/series of used UR robot.",
                choices=[
                    "ur3",
                    "ur3e",
                    "ur5",
                    "ur5e",
                    "ur10",
                    "ur10e",
                    "ur16e",
                    "ur20",
                    "ur30",
                ],
            ),
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
                description="Path where the warehouse database should be stored",
            ),
            DeclareLaunchArgument(
                "launch_servo", default_value="false", description="Launch Servo?"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Using or not time from simulation",
            ),
            DeclareLaunchArgument(
                "publish_robot_description",
                default_value="true",
                description="MoveGroup publishes robot description",
            ),
            DeclareLaunchArgument(
                "publish_robot_description_semantic",
                default_value="true",
                description="MoveGroup publishes robot description semantic",
            ),
            DeclareLaunchArgument(
                "controller_namespace",
                default_value="mur620a",
                description="Namespace where the arm ros2_control controllers run",
            ),
            DeclareLaunchArgument(
                "publish_tf_alias",
                default_value="true",
                description=(
                    "Publish an identity TF from the namespaced base frame to "
                    "MoveIt's unprefixed planning frame"
                ),
            ),
            DeclareLaunchArgument(
                "tf_topic",
                default_value="/tf",
                description="TF topic used by MoveIt and its RViz instance",
            ),
            DeclareLaunchArgument(
                "tf_static_topic",
                default_value="/tf_static",
                description="Static TF topic used by MoveIt and its RViz instance",
            ),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_states",
                description="JointState topic used by MoveIt and its RViz instance",
            ),
            DeclareLaunchArgument(
                "virtual_joint_parent_frame",
                default_value="",
                description=(
                    "Parent frame for MoveIt's fixed virtual joint. Empty uses "
                    "<controller_namespace>/base_footprint."
                ),
            ),
            DeclareLaunchArgument(
                "default_velocity_scaling",
                default_value="0.25",
                description="Default MoveIt velocity scaling factor for generated plans",
            ),
            DeclareLaunchArgument(
                "default_acceleration_scaling",
                default_value="0.15",
                description="Default MoveIt acceleration scaling factor for generated plans",
            ),
        ]
    )


def launch_setup(context, *args, **kwargs):
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_delay = LaunchConfiguration("rviz_delay")
    _ur_type = LaunchConfiguration("ur_type").perform(context)
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path").perform(context)
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    publish_robot_description = ParameterValue(
        LaunchConfiguration("publish_robot_description"), value_type=bool
    )
    publish_robot_description_semantic = ParameterValue(
        LaunchConfiguration("publish_robot_description_semantic"), value_type=bool
    )
    controller_namespace = LaunchConfiguration("controller_namespace").perform(context)
    publish_tf_alias = LaunchConfiguration("publish_tf_alias")
    tf_topic = LaunchConfiguration("tf_topic")
    tf_static_topic = LaunchConfiguration("tf_static_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    default_velocity_scaling = ParameterValue(
        LaunchConfiguration("default_velocity_scaling"),
        value_type=float,
    )
    default_acceleration_scaling = ParameterValue(
        LaunchConfiguration("default_acceleration_scaling"),
        value_type=float,
    )
    virtual_joint_parent_frame = LaunchConfiguration("virtual_joint_parent_frame").perform(context)
    if not virtual_joint_parent_frame:
        virtual_joint_parent_frame = f"{controller_namespace}/base_footprint"
    tf_remappings = [
        ("/tf", tf_topic),
        ("/tf_static", tf_static_topic),
        ("joint_states", joint_states_topic),
    ]

    robot_xacro_file, robot_xacro_mappings = robot_description_source(controller_namespace)
    moveit_config = (
        MoveItConfigsBuilder(robot_name="mur620", package_name="mur_moveit_config")
        .robot_description(robot_xacro_file, robot_xacro_mappings)
        .robot_description_semantic(
            Path("srdf") / "mur620.srdf.xacro",
            {
                "prefix": "UR10",
                "model_name": "mur620",
                "virtual_joint_parent_frame": virtual_joint_parent_frame,
            },
        )
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=controller_namespace,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            arm_controller_config(controller_namespace),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description": publish_robot_description,
                "publish_robot_description_semantic": publish_robot_description_semantic,
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
                "default_velocity_scaling_factor": default_velocity_scaling,
                "default_acceleration_scaling_factor": default_acceleration_scaling,
                "fix_start_state": True,
                "default_workspace_bounds": 100.0,
            },
        ],
        remappings=tf_remappings,
    )

    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node",
        namespace=controller_namespace,
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=tf_remappings,
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name=f"rviz2_{controller_namespace}_moveit",
        output="log",
        arguments=["-d", rviz_config, "--ros-args", "--log-level", "warn"],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=tf_remappings,
    )

    moveit_tf_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="moveit_base_footprint_alias",
        condition=IfCondition(publish_tf_alias),
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            f"{controller_namespace}/base_footprint",
            "--child-frame-id",
            "base_footprint",
        ],
        remappings=tf_remappings,
        output="screen",
    )

    return [
        moveit_tf_alias,
        TimerAction(
            period=2.0,
            actions=[move_group_node, servo_node],
        ),
        TimerAction(
            period=rviz_delay,
            actions=[rviz_node],
        ),
    ]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_entity(declare_arguments())
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
