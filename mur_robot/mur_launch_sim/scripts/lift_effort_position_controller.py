#!/usr/bin/env python3
"""Simple position loop for one MiR/MUR lift joint using an effort controller."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class LiftEffortPositionController(Node):
    def __init__(self, args):
        super().__init__(f'lift_effort_position_controller_{args.arm}')
        self.args = args
        self.joint_name = 'left_lift_joint' if args.arm == 'l' else 'right_lift_joint'
        self.command_topic = (
            args.command_topic
            or f'/{args.robot_name}/lift_controller_{args.arm}/commands'
        )
        self.effort_topic = (
            args.effort_topic
            or f'/{args.robot_name}/lift_effort_controller_{args.arm}/commands'
        )

        self.position = None
        self.velocity = 0.0
        self.target = None
        self.integral = 0.0
        self.last_update = None

        self.effort_pub = self.create_publisher(Float64MultiArray, self.effort_topic, 10)
        self.create_subscription(
            Float64MultiArray,
            self.command_topic,
            self._command_callback,
            10,
        )
        self.create_subscription(
            JointState,
            f'/{args.robot_name}/joint_states',
            self._joint_state_callback,
            10,
        )
        self.create_subscription(JointState, '/joint_states', self._joint_state_callback, 10)
        self.timer = self.create_timer(1.0 / args.rate, self._update)

        self.get_logger().info(
            f'{self.joint_name}: {self.command_topic} -> {self.effort_topic}'
        )

    def _command_callback(self, msg):
        if not msg.data:
            return
        self.target = min(max(float(msg.data[0]), self.args.lower_limit), self.args.upper_limit)
        self.integral = 0.0

    def _joint_state_callback(self, msg):
        try:
            index = msg.name.index(self.joint_name)
        except ValueError:
            return
        if len(msg.position) > index:
            self.position = msg.position[index]
        if len(msg.velocity) > index and math.isfinite(msg.velocity[index]):
            self.velocity = msg.velocity[index]
        if self.target is None and self.position is not None:
            self.target = min(max(self.position, self.args.lower_limit), self.args.upper_limit)

    def _publish_effort(self, effort):
        msg = Float64MultiArray()
        msg.data = [float(effort)]
        self.effort_pub.publish(msg)

    def _update(self):
        now = time.monotonic()
        if self.position is None or self.target is None:
            self.last_update = now
            return

        dt = 1.0 / self.args.rate
        if self.last_update is not None:
            dt = max(1.0e-4, min(now - self.last_update, 0.2))
        self.last_update = now

        error = self.target - self.position
        self.integral += error * dt
        self.integral = min(max(self.integral, -self.args.integral_limit), self.args.integral_limit)

        effort = (
            self.args.gravity_effort
            + self.args.kp * error
            + self.args.ki * self.integral
            - self.args.kd * self.velocity
        )
        effort = min(max(effort, self.args.min_effort), self.args.max_effort)

        if self.position <= self.args.lower_limit + 0.002 and self.target <= self.args.lower_limit:
            effort = min(effort, self.args.gravity_effort)
        if self.position >= self.args.upper_limit - 0.002 and self.target >= self.args.upper_limit:
            effort = max(0.0, min(effort, self.args.gravity_effort))

        self._publish_effort(effort)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620a')
    parser.add_argument('--arm', choices=['l', 'r'], required=True)
    parser.add_argument('--command-topic', default='')
    parser.add_argument('--effort-topic', default='')
    parser.add_argument('--rate', type=float, default=100.0)
    parser.add_argument('--kp', type=float, default=3500.0)
    parser.add_argument('--ki', type=float, default=500.0)
    parser.add_argument('--kd', type=float, default=450.0)
    parser.add_argument('--gravity-effort', type=float, default=335.0)
    parser.add_argument('--min-effort', type=float, default=-200.0)
    parser.add_argument('--max-effort', type=float, default=1200.0)
    parser.add_argument('--integral-limit', type=float, default=0.08)
    parser.add_argument('--lower-limit', type=float, default=0.0)
    parser.add_argument('--upper-limit', type=float, default=0.5)
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = LiftEffortPositionController(args)
    try:
        rclpy.spin(node)
    finally:
        node._publish_effort(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
