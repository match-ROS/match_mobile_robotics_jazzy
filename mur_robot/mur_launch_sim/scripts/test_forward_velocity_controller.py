#!/usr/bin/env python3
"""Send a short, safe command to one UR forward velocity controller."""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


JOINT_COUNT = 6


class ForwardVelocityTester(Node):
    def __init__(self, args):
        super().__init__('test_forward_velocity_controller')
        self.args = args
        if args.command_topic:
            topic = args.command_topic
        else:
            topic = f'/{args.robot_name}/forward_velocity_controller_{args.arm}/commands'
        self.topic = topic
        self.publisher = self.create_publisher(Float64MultiArray, topic, 10)

    def publish_command(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities
        self.publisher.publish(msg)

    def wait_for_controller(self):
        deadline = time.monotonic() + self.args.connect_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                self.get_logger().info(f'Controller command subscriber found on {self.topic}')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn(
            f'No subscriber found on {self.topic} after {self.args.connect_timeout:.1f}s; '
            'publishing anyway'
        )
        return False

    def run_test(self):
        velocities = [0.0] * JOINT_COUNT
        velocities[self.args.joint_index] = self.args.velocity

        self.wait_for_controller()

        period = 1.0 / self.args.rate
        zero = [0.0] * JOINT_COUNT
        settle_samples = int(self.args.settle_time * self.args.rate)
        for _ in range(settle_samples):
            self.publish_command(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.get_logger().info(
            f'Publishing {velocities} to {self.topic} for {self.args.duration:.2f}s'
        )
        end_time = time.monotonic() + self.args.duration
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_command(velocities)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.get_logger().info('Stopping arm with zero velocity command')
        for _ in range(max(5, int(self.args.rate * 0.25))):
            self.publish_command(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620a')
    parser.add_argument('--arm', choices=['l', 'r'], default='l')
    parser.add_argument('--command-topic', default='')
    parser.add_argument('--joint-index', type=int, default=5, choices=range(JOINT_COUNT))
    parser.add_argument('--velocity', type=float, default=0.15)
    parser.add_argument('--duration', type=float, default=2.0)
    parser.add_argument('--rate', type=float, default=50.0)
    parser.add_argument('--connect-timeout', type=float, default=5.0)
    parser.add_argument('--settle-time', type=float, default=0.2)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ForwardVelocityTester(args)
    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
