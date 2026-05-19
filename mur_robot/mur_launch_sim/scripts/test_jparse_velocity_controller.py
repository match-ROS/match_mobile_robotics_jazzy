#!/usr/bin/env python3
"""Send a short Cartesian twist command to the J-PARSE velocity controller."""

import argparse
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class JParseVelocityTester(Node):
    def __init__(self, args):
        super().__init__('test_jparse_velocity_controller')
        self.args = args
        self.topic = (
            args.topic
            or f'/{args.robot_name}/jparse_velocity_controller_{args.arm}/twist_cmd'
        )
        self.base_link = args.base_link or (
            'UR10_l/base_link' if args.arm == 'l' else 'UR10_r/base_link'
        )
        self.publisher = self.create_publisher(TwistStamped, self.topic, 10)

    def publish_twist(self, values):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_link
        msg.twist.linear.x = values[0]
        msg.twist.linear.y = values[1]
        msg.twist.linear.z = values[2]
        msg.twist.angular.x = values[3]
        msg.twist.angular.y = values[4]
        msg.twist.angular.z = values[5]
        self.publisher.publish(msg)

    def run(self):
        command = [
            self.args.vx,
            self.args.vy,
            self.args.vz,
            self.args.wx,
            self.args.wy,
            self.args.wz,
        ]
        self.get_logger().info(
            f'Publishing twist {command} in {self.base_link} on {self.topic} '
            f'for {self.args.duration:.2f}s'
        )
        deadline = time.monotonic() + self.args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_twist(command)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / self.args.rate)

        for _ in range(10):
            self.publish_twist([0.0] * 6)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)
        self.get_logger().info('Sent zero twist')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620a')
    parser.add_argument('--arm', choices=['l', 'r'], default='l')
    parser.add_argument('--topic', default='')
    parser.add_argument('--base-link', default='')
    parser.add_argument('--duration', type=float, default=1.0)
    parser.add_argument('--rate', type=float, default=50.0)
    parser.add_argument('--vx', type=float, default=0.0)
    parser.add_argument('--vy', type=float, default=0.0)
    parser.add_argument('--vz', type=float, default=0.02)
    parser.add_argument('--wx', type=float, default=0.0)
    parser.add_argument('--wy', type=float, default=0.0)
    parser.add_argument('--wz', type=float, default=0.0)
    args, _ = parser.parse_known_args()
    return args


def main():
    rclpy.init()
    node = JParseVelocityTester(parse_args())
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
