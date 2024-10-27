#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit.core.robot_interface import RobotInterface
from moveit.planning import MoveItCommander
from moveit.planning import PlanningComponent
from moveit.core import RobotInterface

class MoveToHomeNode(Node):
    def __init__(self):
        super().__init__('move_to_home_node')

        # Initialize MoveIt interfaces for the robot
        self.robot = RobotInterface(node=self, description="robot_description")
        self.moveit_commander = MoveItCommander(node=self, robot_interface=self.robot)

        # Initialize PlanningComponent for each arm
        self.arm_left_group = PlanningComponent("UR_arm_l", self.robot, self.moveit_commander)
        self.arm_right_group = PlanningComponent("UR_arm_r", self.robot, self.moveit_commander)

        self.get_logger().info("MoveToHomeNode initialized and ready.")

    def move_to_home(self):
        # Move the left arm to the "Home_custom" position
        self.get_logger().info("Moving UR_arm_l to Home_custom position...")
        success_left = self.arm_left_group.set_named_target("Home_custom")
        if success_left:
            plan_left = self.arm_left_group.plan()
            self.arm_left_group.execute(plan_left)
            self.get_logger().info("UR_arm_l moved to Home_custom successfully!")
        else:
            self.get_logger().error("Failed to set target for UR_arm_l")

        # Move the right arm to the "Home_custom" position
        self.get_logger().info("Moving UR_arm_r to Home_custom position...")
        success_right = self.arm_right_group.set_named_target("Home_custom")
        if success_right:
            plan_right = self.arm_right_group.plan()
            self.arm_right_group.execute(plan_right)
            self.get_logger().info("UR_arm_r moved to Home_custom successfully!")
        else:
            self.get_logger().error("Failed to set target for UR_arm_r")

def main(args=None):
    rclpy.init(args=args)
    node = MoveToHomeNode()
    node.move_to_home()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
