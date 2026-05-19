#!/usr/bin/env python3
"""Command one lift controller directly and report measured joint motion."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class LiftControllerTester(Node):
    def __init__(self, args):
        super().__init__('test_lift_controller')
        self.args = args
        self.joint_name = 'left_lift_joint' if args.arm == 'l' else 'right_lift_joint'
        self.controller_name = (
            f'lift_effort_controller_{args.arm}'
            if args.mode == 'effort'
            else f'lift_controller_{args.arm}'
        )
        self.command_topic = (
            args.command_topic
            or f'/{args.robot_name}/{self.controller_name}/commands'
        )
        self.joint_state_topics = [
            args.joint_states_topic,
            f'/{args.robot_name}/joint_states',
            '/joint_states',
        ]
        self.last_position = None
        self.last_stamp = None

        self.publisher = self.create_publisher(
            Float64MultiArray,
            self.command_topic,
            10,
        )
        self.joint_state_subscriptions = [
            self.create_subscription(
                JointState,
                topic,
                self._joint_state_callback,
                10,
            )
            for topic in dict.fromkeys(self.joint_state_topics)
            if topic
        ]

    def _joint_state_callback(self, msg):
        try:
            index = msg.name.index(self.joint_name)
        except ValueError:
            return
        if len(msg.position) <= index:
            return
        self.last_position = msg.position[index]
        self.last_stamp = time.monotonic()

    def _spin_until_position(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_position is not None:
                return self.last_position
        return None

    def _publish_position(self, position):
        msg = Float64MultiArray()
        msg.data = [float(position)]
        self.publisher.publish(msg)

    def wait_for_controller(self):
        deadline = time.monotonic() + self.args.connect_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                self.get_logger().info(
                    f'Controller command subscriber found on {self.command_topic}'
                )
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn(
            f'No subscriber found on {self.command_topic} after '
            f'{self.args.connect_timeout:.1f}s; publishing anyway'
        )
        return False

    def run_test(self):
        target = min(max(self.args.target, 0.0), 0.5)
        self.wait_for_controller()

        initial = self._spin_until_position(self.args.state_timeout)
        if initial is None:
            self.get_logger().error(
                f'No {self.joint_name} sample received on joint state topics: '
                f'{", ".join(topic for topic in self.joint_state_topics if topic)}'
            )
            return 2

        if self.args.mode == 'effort':
            command_value = abs(self.args.effort)
            if target < initial:
                command_value = -command_value
            self.get_logger().info(
                f'{self.joint_name} initial position: {initial:.5f} m; '
                f'commanding {command_value:.2f} N on {self.command_topic}'
            )
        else:
            command_value = target
            self.get_logger().info(
                f'{self.joint_name} initial position: {initial:.5f} m; '
                f'commanding {target:.5f} m on {self.command_topic}'
            )

        period = 1.0 / self.args.rate
        deadline = time.monotonic() + self.args.timeout
        reached = False
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_position(command_value)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.last_position is not None:
                if self.args.mode == 'effort':
                    if abs(self.last_position - initial) >= self.args.min_motion:
                        reached = True
                        break
                else:
                    error = abs(self.last_position - target)
                    if error <= self.args.tolerance:
                        reached = True
                        break
            time.sleep(period)

        final = self.last_position
        if self.args.mode == 'effort':
            self._publish_position(0.0)

        if self.args.restore:
            self.get_logger().info(f'Restoring {self.joint_name} to {initial:.5f} m')
            restore_deadline = time.monotonic() + self.args.timeout
            while rclpy.ok() and time.monotonic() < restore_deadline:
                self._publish_position(initial)
                rclpy.spin_once(self, timeout_sec=0.0)
                if self.last_position is not None:
                    if abs(self.last_position - initial) <= self.args.tolerance:
                        break
                time.sleep(period)

        if final is None or not math.isfinite(final):
            self.get_logger().error('No finite final lift position received')
            return 2

        moved = abs(final - initial)
        target_error = abs(final - target)
        self.get_logger().info(
            f'{self.joint_name} final position: {final:.5f} m; '
            f'moved {moved:.5f} m; target error {target_error:.5f} m'
        )

        if reached:
            self.get_logger().info('Lift controller test PASSED')
            return 0

        if moved < self.args.min_motion:
            self.get_logger().error(
                'Lift controller test FAILED: joint state did not move enough. '
                'This points to the lift controller, ros2_control interface, or Gazebo joint.'
            )
        else:
            self.get_logger().error(
                'Lift controller test FAILED: joint moved, but did not reach target.'
            )
        return 1


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620a')
    parser.add_argument('--arm', choices=['l', 'r'], default='l')
    parser.add_argument('--mode', choices=['position', 'effort'], default='position')
    parser.add_argument('--target', type=float, default=0.2)
    parser.add_argument('--effort', type=float, default=700.0)
    parser.add_argument('--tolerance', type=float, default=0.01)
    parser.add_argument('--min-motion', type=float, default=0.02)
    parser.add_argument('--timeout', type=float, default=8.0)
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--connect-timeout', type=float, default=5.0)
    parser.add_argument('--state-timeout', type=float, default=5.0)
    parser.add_argument('--command-topic', default='')
    parser.add_argument('--joint-states-topic', default='')
    parser.add_argument('--restore', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = LiftControllerTester(args)
    try:
        return_code = node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(return_code)


if __name__ == '__main__':
    main()
