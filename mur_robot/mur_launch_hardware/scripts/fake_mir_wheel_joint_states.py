#!/usr/bin/env python3
"""Publish static MiR wheel joint states for visualization only."""

from sensor_msgs.msg import JointState

import rclpy
from rclpy.node import Node


DEFAULT_JOINT_NAMES = [
    'left_wheel_joint',
    'right_wheel_joint',
    'fl_caster_rotation_joint',
    'fl_caster_wheel_joint',
    'fr_caster_rotation_joint',
    'fr_caster_wheel_joint',
    'bl_caster_rotation_joint',
    'bl_caster_wheel_joint',
    'br_caster_rotation_joint',
    'br_caster_wheel_joint',
]


class FakeMirWheelJointStates(Node):
    def __init__(self):
        super().__init__('fake_mir_wheel_joint_states')

        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('joint_states_topic', '/joint_states')

        self.joint_names = [
            str(name)
            for name in self.get_parameter('joint_names').value
        ]
        frequency = float(self.get_parameter('publish_frequency').value)
        topic = str(self.get_parameter('joint_states_topic').value)

        if frequency <= 0.0:
            raise ValueError('publish_frequency must be greater than zero')
        if not self.joint_names:
            raise ValueError('joint_names must not be empty')

        self.publisher = self.create_publisher(JointState, topic, 10)
        self.timer = self.create_timer(1.0 / frequency, self.publish_joint_states)

        self.get_logger().info(
            f'Publishing fake MiR wheel joint states for {len(self.joint_names)} joints on {topic}.')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FakeMirWheelJointStates()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
