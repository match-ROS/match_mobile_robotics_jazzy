#!/usr/bin/env python3

import math
from statistics import mean

import rclpy
from ewellix_interfaces.msg import State
from rclpy.node import Node
from sensor_msgs.msg import JointState


class EwellixDualStateToJointState(Node):
    def __init__(self):
        super().__init__('ewellix_dual_state_to_joint_state')

        self.declare_parameter('robot_name', 'mur620')
        self.declare_parameter('joint_count', 2)
        self.declare_parameter('conversion', 3225.0)
        self.declare_parameter('position_multiplier', 1.0)
        self.declare_parameter('publish_frequency', 20.0)
        self.declare_parameter('state_timeout', 1.0)
        self.declare_parameter('hold_last_state', True)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('left_state_topic', '')
        self.declare_parameter('right_state_topic', '')

        self.robot_name = self.get_parameter('robot_name').value
        self.joint_count = int(self.get_parameter('joint_count').value)
        self.conversion = float(self.get_parameter('conversion').value)
        self.position_multiplier = float(self.get_parameter('position_multiplier').value)
        self.state_timeout = float(self.get_parameter('state_timeout').value)
        self.hold_last_state = bool(self.get_parameter('hold_last_state').value)
        publish_frequency = float(self.get_parameter('publish_frequency').value)
        joint_states_topic = self.get_parameter('joint_states_topic').value

        left_state_topic = self.get_parameter('left_state_topic').value
        right_state_topic = self.get_parameter('right_state_topic').value
        if not left_state_topic:
            left_state_topic = f'/{self.robot_name}/ewellix_lift_l/state'
        if not right_state_topic:
            right_state_topic = f'/{self.robot_name}/ewellix_lift_r/state'

        self.latest = {
            'left_lift_joint': {
                'position': None,
                'velocity': math.nan,
                'stamp': None,
                'last_position': None,
                'last_stamp': None,
                'warned_missing': False,
            },
            'right_lift_joint': {
                'position': None,
                'velocity': math.nan,
                'stamp': None,
                'last_position': None,
                'last_stamp': None,
                'warned_missing': False,
            },
        }

        self.publisher = self.create_publisher(JointState, joint_states_topic, 10)
        self.left_subscription = self.create_subscription(
            State, left_state_topic, lambda msg: self.state_callback('left_lift_joint', msg), 10)
        self.right_subscription = self.create_subscription(
            State, right_state_topic, lambda msg: self.state_callback('right_lift_joint', msg), 10)

        period = 1.0 / publish_frequency if publish_frequency > 0.0 else 0.05
        self.timer = self.create_timer(period, self.publish_joint_states)
        self.diagnostic_timer = self.create_timer(2.0, self.publish_diagnostics)

        self.get_logger().info(
            f'Publishing lift joint states to {joint_states_topic}; '
            f'left={left_state_topic} -> left_lift_joint, '
            f'right={right_state_topic} -> right_lift_joint'
        )

    def state_callback(self, joint_name, state):
        positions = [
            position for position in state.actual_positions[:self.joint_count]
            if position >= 0
        ]
        if not positions or self.conversion == 0.0:
            self.get_logger().warn(
                f'Ignoring invalid {joint_name} Ewellix state: '
                f'positions={list(state.actual_positions[:self.joint_count])}',
                throttle_duration_sec=2.0,
            )
            return

        now = self.get_clock().now()
        position = mean(positions) / self.conversion * self.position_multiplier

        entry = self.latest[joint_name]
        velocity = math.nan
        if entry['last_stamp'] is not None and entry['last_position'] is not None:
            dt = (now - entry['last_stamp']).nanoseconds * 1e-9
            if dt > 0.0:
                velocity = (position - entry['last_position']) / dt

        first_sample = entry['position'] is None
        entry['position'] = position
        entry['velocity'] = velocity
        entry['stamp'] = now
        entry['last_position'] = position
        entry['last_stamp'] = now

        if first_sample:
            self.get_logger().info(
                f'Received first {joint_name} state: '
                f'raw={list(state.actual_positions[:self.joint_count])}, '
                f'position={position:.5f} m'
            )

    def publish_joint_states(self):
        now = self.get_clock().now()
        msg = JointState()
        msg.header.stamp = now.to_msg()

        for joint_name in ('left_lift_joint', 'right_lift_joint'):
            entry = self.latest[joint_name]
            if entry['position'] is None:
                continue

            age = (now - entry['stamp']).nanoseconds * 1e-9 if entry['stamp'] is not None else math.inf
            if age > self.state_timeout and not self.hold_last_state:
                continue

            msg.name.append(joint_name)
            msg.position.append(entry['position'])
            if not math.isnan(entry['velocity']):
                msg.velocity.append(entry['velocity'])

        if msg.name:
            if msg.velocity and len(msg.velocity) != len(msg.name):
                msg.velocity = []
            self.publisher.publish(msg)

    def publish_diagnostics(self):
        now = self.get_clock().now()
        for joint_name, entry in self.latest.items():
            if entry['position'] is None:
                self.get_logger().warn(
                    f'No Ewellix state received yet for {joint_name}',
                    throttle_duration_sec=10.0,
                )
                continue

            age = (now - entry['stamp']).nanoseconds * 1e-9 if entry['stamp'] is not None else math.inf
            if age > self.state_timeout:
                self.get_logger().warn(
                    f'{joint_name} Ewellix state is stale by {age:.2f}s; '
                    f'hold_last_state={self.hold_last_state}',
                    throttle_duration_sec=5.0,
                )


def main():
    rclpy.init()
    node = EwellixDualStateToJointState()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
