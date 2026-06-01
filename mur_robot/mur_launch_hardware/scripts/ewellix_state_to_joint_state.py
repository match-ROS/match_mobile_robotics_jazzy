#!/usr/bin/env python3

import math
from statistics import mean

import rclpy
from ewellix_interfaces.msg import State
from rclpy.node import Node
from sensor_msgs.msg import JointState


class EwellixStateToJointState(Node):
    def __init__(self):
        super().__init__('ewellix_state_to_joint_state')

        self.declare_parameter('joint_name', 'lift_joint')
        self.declare_parameter('conversion', 3225.0)
        self.declare_parameter('position_multiplier', 1.0)
        self.declare_parameter('joint_states_topic', '/joint_states')

        self.joint_name = self.get_parameter('joint_name').value
        self.conversion = float(self.get_parameter('conversion').value)
        self.position_multiplier = float(self.get_parameter('position_multiplier').value)

        joint_states_topic = self.get_parameter('joint_states_topic').value
        self.last_stamp = None
        self.last_position = None

        self.publisher = self.create_publisher(JointState, joint_states_topic, 10)
        self.subscription = self.create_subscription(State, 'state', self.state_callback, 10)

    def state_callback(self, state):
        positions = [position for position in state.actual_positions if position >= 0]
        if not positions or self.conversion == 0.0:
            return

        now = self.get_clock().now()
        position = mean(positions) / self.conversion * self.position_multiplier

        velocity = math.nan
        if self.last_stamp is not None and self.last_position is not None:
            dt = (now - self.last_stamp).nanoseconds * 1e-9
            if dt > 0.0:
                velocity = (position - self.last_position) / dt

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [self.joint_name]
        msg.position = [position]
        if not math.isnan(velocity):
            msg.velocity = [velocity]

        self.publisher.publish(msg)
        self.last_stamp = now
        self.last_position = position


def main():
    rclpy.init()
    node = EwellixStateToJointState()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
