#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import time
import subprocess, sys



class MoveToHomeNode(Node):
    def __init__(self):
        super().__init__('moveit_move_to_home_node')

        # Initialize MoveItPy with the node's name
        self.declare_parameter('robot_description', '')
        self.declare_parameter('robot_description_semantic', '')
        self.declare_parameter('moveit_cpp', '')

        moveit_config_builder = MoveItConfigsBuilder("mur620", package_name="mur_moveit_config")
        moveit_config_builder.robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("mur_moveit_config"),
                "srdf",
                "mur620.srdf.xacro"
            ),
            mappings={"model_name": "mur620", "prefix": "UR10"}
        )
        moveit_config_builder.moveit_cpp(file_path=os.path.join(
            get_package_share_directory("mur_moveit_config"),
            "config",
            "moveit_cpp.yaml"
        ))
        moveit_config_dict = moveit_config_builder.to_moveit_configs().to_dict()

        self.moveit = MoveItPy(node_name="Tobias_cool_movit_planner", config_dict=moveit_config_dict)
       
        time.sleep(1)  # Wait for the MoveItPy node to initialize
        # Set the use_sim_time parameter to true
        command = "ros2 param set /Tobias_cool_movit_planner use_sim_time true"

        # this is a hack to run the command in bash - ideally we should use ROS2 API to set the parameter (but I don't know how)
        subprocess.run(command, shell = True, executable="/bin/bash")

        # Create move group for each arm
        self.arm_left_group = self.moveit.get_planning_component("UR_arm_l")
        self.arm_right_group = self.moveit.get_planning_component("UR_arm_r")

    def move_to_home(self):
        # Move the left arm to the "Home_custom" named target
        self.get_logger().info("Moving UR_arm_l to Home_custom position...")
        if self.arm_left_group.set_goal_state("Home_custom"):
            plan_left = self.arm_left_group.plan()
            if plan_left.error_code.val == 1:  # In ROS, error_code.val == 1 indicates success
                self.moveit.execute(plan_left.trajectory,controllers=[])
                self.get_logger().info("UR_arm_l moved to Home_custom successfully!")
            else:
                self.get_logger().error(f"Planning failed for UR_arm_l with error code: {plan_left.error_code.val}")
        else:
            self.get_logger().error("Setting goal failed for UR_arm_l")

        # Move the right arm to the "Home_custom" named target
        self.get_logger().info("Moving UR_arm_r to Home_custom position...")
        if self.arm_right_group.set_goal_state("Home_custom"):
            plan_right = self.arm_right_group.plan()
            if plan_right.error_code.val == 1:
                self.moveit.execute(plan_right.trajectory,controllers=[])
                self.get_logger().info("UR_arm_r moved to Home_custom successfully!")
            else:
                self.get_logger().error(f"Planning failed for UR_arm_r with error code: {plan_right.error_code.val}")
        else:
            self.get_logger().error("Setting goal failed for UR_arm_r")


def main(args=None):
    rclpy.init(args=args)
    node = MoveToHomeNode()
    node.move_to_home()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
