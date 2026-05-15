#!/usr/bin/env python3
"""Republish LaserScan messages with the TF frame used by the robot model."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class LaserScanFrameRepublisher(Node):
    def __init__(self):
        super().__init__('laserscan_frame_republisher')

        self.declare_parameter('input_topic', '')
        self.declare_parameter('output_topic', '')
        self.declare_parameter('frame_id', '')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        if not input_topic or not output_topic or not self.frame_id:
            raise ValueError('input_topic, output_topic, and frame_id must be set')

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher = self.create_publisher(LaserScan, output_topic, output_qos)
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            input_qos,
        )

        self.get_logger().info(
            f'Republishing {input_topic} -> {output_topic} with frame_id={self.frame_id}'
        )

    def scan_callback(self, msg):
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = LaserScanFrameRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
