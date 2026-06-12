#!/usr/bin/env python3
"""Send a short equilibrium twist command to the Cartesian admittance controller."""

import argparse
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


AXIS_INDEX = {
    'x': 0,
    'y': 1,
    'z': 2,
    'rx': 3,
    'ry': 4,
    'rz': 5,
}


class CartesianAdmittanceTester(Node):
    def __init__(self, args):
        super().__init__('test_cartesian_admittance_controller')
        self.args = args
        self.topic = (
            args.topic
            or f'/{args.robot_name}/cartesian_admittance_controller_{args.arm}/equilibrium_twist_cmd'
        )
        self.frame_id = args.frame_id or (
            'UR10_l/base_link' if args.arm == 'l' else 'UR10_r/base_link'
        )
        self.publisher = self.create_publisher(TwistStamped, self.topic, 10)

    def wait_for_controller(self):
        deadline = time.monotonic() + self.args.connect_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                self.get_logger().info(
                    f'Cartesian admittance subscriber found on {self.topic}'
                )
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn(
            f'No subscriber found on {self.topic} after '
            f'{self.args.connect_timeout:.1f}s; publishing anyway'
        )
        return False

    def publish_twist(self, values):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = values[0]
        msg.twist.linear.y = values[1]
        msg.twist.linear.z = values[2]
        msg.twist.angular.x = values[3]
        msg.twist.angular.y = values[4]
        msg.twist.angular.z = values[5]
        self.publisher.publish(msg)

    def run_test(self):
        values = [0.0] * 6
        values[AXIS_INDEX[self.args.axis]] = self.args.velocity
        zero = [0.0] * 6

        self.wait_for_controller()
        period = 1.0 / self.args.rate
        self.get_logger().info(
            f'Publishing {self.args.axis}={self.args.velocity:.4f} in {self.frame_id} '
            f'to {self.topic} for {self.args.duration:.2f}s'
        )

        end_time = time.monotonic() + self.args.duration
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_twist(values)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.get_logger().info('Stopping equilibrium motion')
        for _ in range(max(5, int(self.args.rate * 0.25))):
            self.publish_twist(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620')
    parser.add_argument('--arm', choices=['l', 'r'], default='r')
    parser.add_argument('--topic')
    parser.add_argument('--frame-id')
    parser.add_argument('--axis', choices=sorted(AXIS_INDEX), default='x')
    parser.add_argument('--velocity', type=float, default=0.02)
    parser.add_argument('--duration', type=float, default=2.0)
    parser.add_argument('--rate', type=float, default=100.0)
    parser.add_argument('--connect-timeout', type=float, default=5.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = CartesianAdmittanceTester(args)
    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
