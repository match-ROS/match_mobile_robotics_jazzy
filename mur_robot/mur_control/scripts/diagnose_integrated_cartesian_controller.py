#!/usr/bin/env python3
"""Diagnose the integrated ros2_control Cartesian arm controller."""

import argparse
import math
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


AXIS_INDEX = {
    'x': 0,
    'y': 1,
    'z': 2,
    'rx': 3,
    'ry': 4,
    'rz': 5,
}


@dataclass
class TopicState:
    name: str
    last_time: float = 0.0
    count: int = 0
    max_abs: float = 0.0
    samples: list = field(default_factory=list)

    def update(self, values):
        self.last_time = time.monotonic()
        self.count += 1
        finite = [abs(value) for value in values if math.isfinite(value)]
        if finite:
            self.max_abs = max(self.max_abs, max(finite))
            self.samples.append(values)
            self.samples = self.samples[-8:]

    def age(self):
        if self.last_time <= 0.0:
            return None
        return time.monotonic() - self.last_time


def twist_values(msg):
    return [
        msg.twist.linear.x,
        msg.twist.linear.y,
        msg.twist.linear.z,
        msg.twist.angular.x,
        msg.twist.angular.y,
        msg.twist.angular.z,
    ]


def wrench_values(msg):
    return [
        msg.wrench.force.x,
        msg.wrench.force.y,
        msg.wrench.force.z,
        msg.wrench.torque.x,
        msg.wrench.torque.y,
        msg.wrench.torque.z,
    ]


def pose_values(msg):
    return [
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ]


def array_values(msg):
    return list(msg.data)


def fmt(values):
    return '[' + ' '.join(f'{value:.4f}' for value in values) + ']'


class IntegratedCartesianDiagnoser(Node):
    def __init__(self, args):
        super().__init__('diagnose_integrated_cartesian_controller')
        self.args = args
        self.robot_name = args.robot_name
        self.arm = args.arm
        self.arm_name = f'UR10_{self.arm}'
        self.controller_name = args.controller_name
        self.controller_ns = f'/{self.robot_name}/{self.arm_name}/{self.controller_name}'
        self.command_frame = args.command_frame or f'{self.arm_name}/base_link'
        self.log_handle = None
        self.log_path = self._make_log_path()
        if self.log_path is not None:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log_handle = self.log_path.open('w', encoding='utf-8')
            self._print(f'Logging diagnostic output to {self.log_path}')

        self.topics = {
            'input': TopicState(args.input_topic or f'{self.controller_ns}/equilibrium_twist_cmd'),
            'debug_twist': TopicState(args.debug_topic or f'{self.controller_ns}/debug_twist'),
            'singular_values': TopicState(args.singular_topic or f'{self.controller_ns}/singular_values'),
            'filtered_wrench': TopicState(args.filtered_wrench_topic or f'{self.controller_ns}/filtered_wrench'),
            'equilibrium_pose': TopicState(args.equilibrium_pose_topic or f'{self.controller_ns}/equilibrium_pose'),
            'target_pose': TopicState(args.target_pose_topic or f'{self.controller_ns}/target_pose'),
            'joint_states': TopicState(args.joint_states_topic or '/joint_states'),
        }
        self.joint_names = [
            f'{self.arm_name}/shoulder_pan_joint',
            f'{self.arm_name}/shoulder_lift_joint',
            f'{self.arm_name}/elbow_joint',
            f'{self.arm_name}/wrist_1_joint',
            f'{self.arm_name}/wrist_2_joint',
            f'{self.arm_name}/wrist_3_joint',
        ]
        self.joint_positions = {}

        self.create_subscription(TwistStamped, self.topics['input'].name, self._cb('input', twist_values), 10)
        self.create_subscription(Float64MultiArray, self.topics['debug_twist'].name, self._cb('debug_twist', array_values), 10)
        self.create_subscription(Float64MultiArray, self.topics['singular_values'].name, self._cb('singular_values', array_values), 10)
        self.create_subscription(WrenchStamped, self.topics['filtered_wrench'].name, self._cb('filtered_wrench', wrench_values), 10)
        self.create_subscription(PoseStamped, self.topics['equilibrium_pose'].name, self._cb('equilibrium_pose', pose_values), 10)
        self.create_subscription(PoseStamped, self.topics['target_pose'].name, self._cb('target_pose', pose_values), 10)
        self.create_subscription(JointState, self.topics['joint_states'].name, self._joint_cb, rclpy.qos.qos_profile_sensor_data)
        self.test_pub = self.create_publisher(TwistStamped, self.topics['input'].name, 10)

    def _make_log_path(self):
        if self.args.log_file == 'none':
            return None
        if self.args.log_file:
            return Path(self.args.log_file).expanduser()
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        name = f'integrated_cartesian_{self.args.robot_name}_{self.args.arm}_{stamp}.log'
        return Path(self.args.log_dir).expanduser() / name

    def _print(self, *parts):
        text = ' '.join(str(part) for part in parts)
        print(text)
        if self.log_handle is not None:
            self.log_handle.write(text + '\n')
            self.log_handle.flush()

    def _cb(self, key, extractor):
        def callback(msg):
            self.topics[key].update(extractor(msg))
        return callback

    def _joint_cb(self, msg):
        values = []
        for index, name in enumerate(msg.name):
            if name not in self.joint_names:
                continue
            if index < len(msg.position) and math.isfinite(msg.position[index]):
                self.joint_positions[name] = msg.position[index]
                values.append(msg.position[index])
        self.topics['joint_states'].update(values)

    def publish_test_twist(self, active):
        values = [0.0] * 6
        if active:
            values[AXIS_INDEX[self.args.axis]] = self.args.velocity
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.twist.linear.x = values[0]
        msg.twist.linear.y = values[1]
        msg.twist.linear.z = values[2]
        msg.twist.angular.x = values[3]
        msg.twist.angular.y = values[4]
        msg.twist.angular.z = values[5]
        self.test_pub.publish(msg)

    def run(self):
        self.print_overview()
        start = time.monotonic()
        end = start + self.args.duration
        test_end = start + self.args.test_duration
        next_tick = start
        if self.args.send_test_command:
            self.get_logger().info(
                f'Sending test twist {self.args.axis}={self.args.velocity:.4f} for '
                f'{self.args.test_duration:.2f}s to {self.topics["input"].name}'
            )
        while rclpy.ok() and time.monotonic() < end:
            now = time.monotonic()
            if self.args.send_test_command:
                self.publish_test_twist(now < test_end)
            rclpy.spin_once(self, timeout_sec=0.01)
            if now >= next_tick:
                self.print_tick()
                next_tick = now + 1.0
        if self.args.send_test_command:
            for _ in range(10):
                self.publish_test_twist(False)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.02)
        self.print_summary()
        if self.log_path is not None:
            self._print(f'\nLog file: {self.log_path}')

    def print_overview(self):
        self._print('\n=== Integrated Cartesian Controller Diagnostic ===')
        self._print(f'robot={self.robot_name} arm={self.arm} controller={self.controller_name}')
        self._print(f'controller_ns={self.controller_ns}')
        self._print(f'command_frame={self.command_frame}')
        self._print('\n--- Topic graph ---')
        for label, state in self.topics.items():
            pubs = self.get_publishers_info_by_topic(state.name)
            subs = self.get_subscriptions_info_by_topic(state.name)
            pub_types = sorted({info.topic_type for info in pubs})
            sub_types = sorted({info.topic_type for info in subs})
            self._print(
                f'{label:18s} pubs={len(pubs):2d} subs={len(subs):2d} '
                f'{state.name} pub_types={pub_types or "-"} sub_types={sub_types or "-"}'
            )
        self.print_controllers()

    def print_controllers(self):
        service = f'/{self.robot_name}/{self.arm_name}/controller_manager/list_controllers'
        client = self.create_client(ListControllers, service)
        if not client.wait_for_service(timeout_sec=self.args.controller_timeout):
            self._print(f'\nController manager not reachable: {service}')
            return
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.controller_timeout)
        if not future.done() or future.result() is None:
            self._print(f'\nController manager did not answer: {service}')
            return
        self._print('\n--- Controllers ---')
        for controller in future.result().controller:
            if controller.name in (
                self.controller_name,
                'forward_velocity_controller',
                'scaled_joint_trajectory_controller',
                'joint_trajectory_controller',
                'force_torque_sensor_broadcaster',
                'joint_state_broadcaster',
            ):
                self._print(f'  {controller.name:38s} {controller.state}')

    def print_tick(self):
        def brief(key):
            state = self.topics[key]
            age = state.age()
            if age is None:
                return f'{key}=none'
            return f'{key}=count:{state.count} age:{age:.2f}s max:{state.max_abs:.4g}'

        self._print(
            'tick ' + ', '.join(
                brief(key)
                for key in ('input', 'debug_twist', 'singular_values', 'filtered_wrench', 'joint_states')
            )
        )

    def print_summary(self):
        self._print('\n=== Summary ===')
        for label, state in self.topics.items():
            age = state.age()
            age_text = 'never' if age is None else f'{age:.3f}s ago'
            self._print(
                f'{label:18s} count={state.count:5d} last={age_text:>12s} '
                f'max_abs={state.max_abs:.6g} topic={state.name}'
            )
            if state.samples:
                self._print(f'  last values: {fmt(state.samples[-1])}')
        missing = [name for name in self.joint_names if name not in self.joint_positions]
        if missing:
            self._print('\nMissing joint states:')
            for name in missing:
                self._print(f'  {name}')
        self._print('\n--- Interpretation hints ---')
        if self.topics['debug_twist'].count == 0:
            self._print('* No debug_twist received. Check whether integrated_cartesian_arm_controller is active.')
        elif self.args.send_test_command and self.topics['debug_twist'].max_abs < 1.0e-6:
            self._print('* Test command was sent, but debug output stayed zero. Check command topic and controller state.')
        elif self.args.send_test_command:
            self._print('* Integrated controller produced nonzero debug output for the test command.')
        if self.topics['joint_states'].count == 0:
            self._print('* No joint_states received; the controller cannot compute FK/Jacobian without state interfaces.')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620')
    parser.add_argument('--arm', choices=['l', 'r'], default='r')
    parser.add_argument('--controller-name', default='integrated_cartesian_arm_controller')
    parser.add_argument('--duration', type=float, default=6.0)
    parser.add_argument('--send-test-command', action='store_true')
    parser.add_argument('--test-duration', type=float, default=2.0)
    parser.add_argument('--axis', choices=sorted(AXIS_INDEX), default='x')
    parser.add_argument('--velocity', type=float, default=0.01)
    parser.add_argument('--command-frame')
    parser.add_argument('--input-topic')
    parser.add_argument('--debug-topic')
    parser.add_argument('--singular-topic')
    parser.add_argument('--filtered-wrench-topic')
    parser.add_argument('--equilibrium-pose-topic')
    parser.add_argument('--target-pose-topic')
    parser.add_argument('--joint-states-topic')
    parser.add_argument('--controller-timeout', type=float, default=2.0)
    parser.add_argument('--log-dir', default='~/integrated_cartesian_diagnostics')
    parser.add_argument(
        '--log-file',
        default='',
        help="Log file path. Default: auto file in --log-dir. Use 'none' to disable.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = IntegratedCartesianDiagnoser(args)
    try:
        node.run()
    finally:
        if node.log_handle is not None:
            node.log_handle.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
