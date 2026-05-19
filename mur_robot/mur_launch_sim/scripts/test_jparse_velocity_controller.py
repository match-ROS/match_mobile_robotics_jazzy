#!/usr/bin/env python3
"""Send a short Cartesian twist command to the J-PARSE velocity controller."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


def quat_inverse(q):
    return [-q[0], -q[1], -q[2], q[3]]


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def quat_angle(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1.0e-12:
        return 0.0
    qw = max(-1.0, min(1.0, q[3] / norm))
    return 2.0 * math.atan2(
        math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]) / norm,
        abs(qw),
    )


def transform_to_pose(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (
        [translation.x, translation.y, translation.z],
        [rotation.x, rotation.y, rotation.z, rotation.w],
    )


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
        self.tip_link = args.tip_link or (
            'UR10_l/tool0' if args.arm == 'l' else 'UR10_r/tool0'
        )
        self.tf_base_frame = args.tf_base_frame or f'{args.robot_name}/{self.base_link}'
        self.tf_tip_frame = args.tf_tip_frame or f'{args.robot_name}/{self.tip_link}'
        self.joint_names = [
            f'UR10_{args.arm}/shoulder_pan_joint',
            f'UR10_{args.arm}/shoulder_lift_joint',
            f'UR10_{args.arm}/elbow_joint',
            f'UR10_{args.arm}/wrist_1_joint',
            f'UR10_{args.arm}/wrist_2_joint',
            f'UR10_{args.arm}/wrist_3_joint',
        ]
        self.publisher = self.create_publisher(TwistStamped, self.topic, 10)
        self.debug_topic = (
            args.debug_topic
            or f'/{args.robot_name}/jparse_velocity_controller_{args.arm}/debug_twist'
        )
        self.last_debug = None
        self.last_motion_debug = None
        self.last_command = None
        self.last_motion_command = None
        self.last_joint_positions = {}
        self.last_joint_velocities = {}
        self.create_subscription(
            Float64MultiArray,
            self.debug_topic,
            self._debug_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            f'/{args.robot_name}/forward_velocity_controller_{args.arm}/commands',
            self._command_callback,
            10,
        )
        self.create_subscription(
            JointState,
            f'/{args.robot_name}/joint_states',
            self._joint_state_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _debug_callback(self, msg):
        self.last_debug = list(msg.data)
        if len(self.last_debug) >= 7 and any(abs(value) > 1.0e-6 for value in self.last_debug[1:7]):
            self.last_motion_debug = self.last_debug

    def _command_callback(self, msg):
        self.last_command = list(msg.data)
        if any(abs(value) > 1.0e-6 for value in self.last_command):
            self.last_motion_command = self.last_command

    def _joint_state_callback(self, msg):
        for index, name in enumerate(msg.name):
            if name not in self.joint_names:
                continue
            if index < len(msg.position) and math.isfinite(msg.position[index]):
                self.last_joint_positions[name] = msg.position[index]
            if index < len(msg.velocity) and math.isfinite(msg.velocity[index]):
                self.last_joint_velocities[name] = msg.velocity[index]

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

    def wait_for_controller(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                self.get_logger().info(f'J-PARSE twist subscriber found on {self.topic}')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn(
            f'No J-PARSE subscriber found on {self.topic} after {timeout:.1f}s; publishing anyway'
        )
        return False

    def lookup_tool_pose(self, timeout=2.0):
        deadline = time.monotonic() + timeout
        last_error = None
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.tf_base_frame,
                    self.tf_tip_frame,
                    rclpy.time.Time(),
                )
                return transform_to_pose(transform)
            except TransformException as exc:
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.02)
        raise RuntimeError(
            f'Could not lookup TF {self.tf_base_frame} -> {self.tf_tip_frame}: {last_error}'
        )

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
        self.wait_for_controller()
        try:
            start_position, start_orientation = self.lookup_tool_pose()
        except RuntimeError as exc:
            self.get_logger().warn(str(exc))
            start_position = None
            start_orientation = None
        self.wait_for_joint_states(timeout=1.0)
        start_joint_positions = self.joint_positions_snapshot()

        deadline = time.monotonic() + self.args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_twist(command)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / self.args.rate)

        try:
            end_position, end_orientation = self.lookup_tool_pose(timeout=0.5)
        except RuntimeError as exc:
            self.get_logger().warn(str(exc))
            end_position = None
            end_orientation = None
        self.wait_for_joint_states(timeout=0.5)
        end_joint_positions = self.joint_positions_snapshot()

        for _ in range(10):
            self.publish_twist([0.0] * 6)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)
        self.get_logger().info('Sent zero twist')
        if start_position is not None and end_position is not None:
            delta = [
                end_position[index] - start_position[index]
                for index in range(3)
            ]
            q_delta = quat_multiply(end_orientation, quat_inverse(start_orientation))
            angle = quat_angle(q_delta)
            measured_velocity = [
                value / max(self.args.duration, 1.0e-9)
                for value in delta
            ]
            self.get_logger().info(
                'Measured TF motion %s -> %s: delta=[%.4f %.4f %.4f] m, '
                'avg_linear=[%.4f %.4f %.4f] m/s, orientation_delta=%.3f deg' % (
                    self.tf_base_frame,
                    self.tf_tip_frame,
                    delta[0], delta[1], delta[2],
                    measured_velocity[0], measured_velocity[1], measured_velocity[2],
                    math.degrees(angle),
                )
            )
        if all(math.isfinite(value) for value in start_joint_positions + end_joint_positions):
            joint_delta = [
                end_joint_positions[index] - start_joint_positions[index]
                for index in range(len(self.joint_names))
            ]
            joint_avg = [
                value / max(self.args.duration, 1.0e-9)
                for value in joint_delta
            ]
            self.get_logger().info(
                'Start joint positions rad: [%s]' % ' '.join(
                    f'{value:.4f}' for value in start_joint_positions
                )
            )
            self.get_logger().info(
                'End joint positions rad:   [%s]' % ' '.join(
                    f'{value:.4f}' for value in end_joint_positions
                )
            )
            self.get_logger().info(
                'Measured joint delta rad: [%s]' % ' '.join(
                    f'{value:.4f}' for value in joint_delta
                )
            )
            self.get_logger().info(
                'Measured avg joint velocity rad/s: [%s]' % ' '.join(
                    f'{value:.4f}' for value in joint_avg
                )
            )
            command_debug = self.last_motion_command or self.last_command
            if command_debug and len(command_debug) == len(joint_avg):
                tracking_error = [
                    command_debug[index] - joint_avg[index]
                    for index in range(len(joint_avg))
                ]
                self.get_logger().info(
                    'Velocity tracking error cmd-measured rad/s: [%s]' % ' '.join(
                        f'{value:.4f}' for value in tracking_error
                    )
                )
                bad_joints = [
                    self.joint_names[index]
                    for index, error in enumerate(tracking_error)
                    if abs(command_debug[index]) > 0.02 and abs(error) > 0.05
                ]
                if bad_joints:
                    self.get_logger().warn(
                        'These joints did not track the forwarded velocity well: %s'
                        % ', '.join(bad_joints)
                    )
        command_debug = self.last_motion_command or self.last_command
        if command_debug:
            self.get_logger().info(
                'Last forwarded velocity command rad/s: [%s]' % ' '.join(
                    f'{value:.4f}' for value in command_debug
                )
            )
        debug = self.last_motion_debug or self.last_debug
        if debug and len(debug) >= 13:
            desired = debug[1:7]
            achieved = debug[7:13]
            qdot = debug[13:]
            self.get_logger().info(
                'Last IDK debug: inverse_condition=%.5f desired=[%.4f %.4f %.4f %.4f %.4f %.4f] '
                'achieved=[%.4f %.4f %.4f %.4f %.4f %.4f]' % (
                    debug[0],
                    desired[0], desired[1], desired[2], desired[3], desired[4], desired[5],
                    achieved[0], achieved[1], achieved[2],
                    achieved[3], achieved[4], achieved[5],
                )
            )
            if qdot:
                self.get_logger().info(
                    'Last IDK qdot rad/s: [%s]' % ' '.join(
                        f'{value:.4f}' for value in qdot
                    )
                )
        else:
            self.get_logger().warn(
                f'No debug message received from {self.debug_topic}. Is the J-PARSE node running?'
            )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620a')
    parser.add_argument('--arm', choices=['l', 'r'], default='l')
    parser.add_argument('--topic', default='')
    parser.add_argument('--debug-topic', default='')
    parser.add_argument('--base-link', default='')
    parser.add_argument('--tip-link', default='')
    parser.add_argument('--tf-base-frame', default='')
    parser.add_argument('--tf-tip-frame', default='')
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
