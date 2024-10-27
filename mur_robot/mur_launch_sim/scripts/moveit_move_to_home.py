#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy

class MoveToHomeNode(Node):
    def __init__(self):
        super().__init__('move_to_home_node')

        # Initialize MoveItPy with the node's name
        self.moveit = MoveItPy(node_name=self.get_name())

        # Create move group for each arm
        self.arm_left_group = self.moveit.get_planning_component("UR_arm_l")
        self.arm_right_group = self.moveit.get_planning_component("UR_arm_r")

        self.get_logger().info("MoveToHomeNode initialized and ready.")

    def move_to_home(self):
        # Move the left arm to the "Home_custom" position
        self.get_logger().info("Moving UR_arm_l to Home_custom position...")
        self.arm_left_group.set_named_target("Home_custom")
        plan_left = self.arm_left_group.plan()
        
        if plan_left.success:
            self.arm_left_group.execute()
            self.get_logger().info("UR_arm_l moved to Home_custom successfully!")
        else:
            self.get_logger().error("Planning failed for UR_arm_l")

        # Move the right arm to the "Home_custom" position
        self.get_logger().info("Moving UR_arm_r to Home_custom position...")
        self.arm_right_group.set_named_target("Home_custom")
        plan_right = self.arm_right_group.plan()

        if plan_right.success:
            self.arm_right_group.execute()
            self.get_logger().info("UR_arm_r moved to Home_custom successfully!")
        else:
            self.get_logger().error("Planning failed for UR_arm_r")

def main(args=None):
    rclpy.init(args=args)
    node = MoveToHomeNode()
    node.move_to_home()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
