#!/usr/bin/env python3
"""Send a short, safe command to one UR forward velocity controller."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
        self.joint_names = [
            f'UR10_{args.arm}/shoulder_pan_joint',
            f'UR10_{args.arm}/shoulder_lift_joint',
            f'UR10_{args.arm}/elbow_joint',
            f'UR10_{args.arm}/wrist_1_joint',
            f'UR10_{args.arm}/wrist_2_joint',
            f'UR10_{args.arm}/wrist_3_joint',
        ]
        self.last_joint_positions = {}
        self.create_subscription(
            JointState,
            f'/{args.robot_name}/joint_states',
            self._joint_state_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

    def _joint_state_callback(self, msg):
        for index, name in enumerate(msg.name):
            if name not in self.joint_names:
                continue
            if index < len(msg.position) and math.isfinite(msg.position[index]):
                self.last_joint_positions[name] = msg.position[index]

    def wait_for_joint_states(self, timeout=2.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if all(name in self.last_joint_positions for name in self.joint_names):
                return True
            rclpy.spin_once(self, timeout_sec=0.02)
        return False

    def joint_positions_snapshot(self):
        return [
            self.last_joint_positions.get(name, float('nan'))
            for name in self.joint_names
        ]

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

        setup_start = time.monotonic()
        self.wait_for_controller()
        self.wait_for_joint_states()
        setup_duration = time.monotonic() - setup_start

        period = 1.0 / self.args.rate
        zero = [0.0] * JOINT_COUNT
        settle_samples = int(self.args.settle_time * self.args.rate)
        for _ in range(settle_samples):
            self.publish_command(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        start_positions = self.joint_positions_snapshot()
        self.get_logger().info(
            f'Publishing {velocities} to {self.topic} for {self.args.duration:.2f}s '
            f'after {setup_duration:.3f}s setup/discovery wait'
        )
        active_start = time.monotonic()
        end_time = time.monotonic() + self.args.duration
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_command(velocities)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        active_duration = time.monotonic() - active_start
        end_positions = self.joint_positions_snapshot()

        self.get_logger().info('Stopping arm with zero velocity command')
        for _ in range(max(5, int(self.args.rate * 0.25))):
            self.publish_command(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        if all(math.isfinite(value) for value in start_positions + end_positions):
            delta = [
                end_positions[index] - start_positions[index]
                for index in range(JOINT_COUNT)
            ]
            self.get_logger().info(
                'Start joint positions rad: [%s]' % ' '.join(
                    f'{value:.4f}' for value in start_positions
                )
            )
            self.get_logger().info(
                'End joint positions rad:   [%s]' % ' '.join(
                    f'{value:.4f}' for value in end_positions
                )
            )
            self.get_logger().info(
                'Measured joint delta rad: [%s]' % ' '.join(
                    f'{value:.4f}' for value in delta
                )
            )
            self.get_logger().info(
                'Measured avg joint velocity rad/s: [%s]' % ' '.join(
                    f'{value / max(active_duration, 1.0e-9):.4f}' for value in delta
                )
            )


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
