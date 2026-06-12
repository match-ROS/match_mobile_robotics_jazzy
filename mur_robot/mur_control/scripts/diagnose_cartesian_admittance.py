#!/usr/bin/env python3
"""Diagnose the Cartesian admittance -> J-PARSE -> joint velocity command chain."""

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
from tf2_ros import Buffer, TransformException, TransformListener


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
    msg_type: str
    last_msg: object = None
    last_time: float = 0.0
    count: int = 0
    max_abs: float = 0.0
    samples: list = field(default_factory=list)

    def update(self, msg, stamp, values):
        self.last_msg = msg
        self.last_time = stamp
        self.count += 1
        abs_values = [abs(value) for value in values if math.isfinite(value)]
        if abs_values:
            self.max_abs = max(self.max_abs, max(abs_values))
            self.samples.append(values)
            self.samples = self.samples[-20:]

    def age(self, now):
        if self.last_time <= 0.0:
            return None
        return now - self.last_time


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


def multi_array_values(msg):
    return list(msg.data)


def fmt_values(values, precision=4):
    return '[' + ' '.join(f'{value:.{precision}f}' for value in values) + ']'


class CartesianAdmittanceDiagnoser(Node):
    def __init__(self, args):
        super().__init__('diagnose_cartesian_admittance')
        self.args = args
        self.log_handle = None
        self.log_path = self._make_log_path(args)
        if self.log_path is not None:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log_handle = self.log_path.open('w', encoding='utf-8')
            self._print(f'Logging diagnostic output to {self.log_path}')
        self.arm_name = f'UR10_{args.arm}'
        self.base_frame = args.base_frame or f'{args.robot_name}/{self.arm_name}/base_link'
        self.tcp_frame = args.tcp_frame or f'{args.robot_name}/{self.arm_name}/tool0'
        self.command_frame = args.command_frame or f'{self.arm_name}/base_link'

        self.topics = {
            'input': TopicState(
                args.input_topic
                or f'/{args.robot_name}/cartesian_admittance_controller_{args.arm}/equilibrium_twist_cmd',
                'geometry_msgs/msg/TwistStamped',
            ),
            'admittance_output': TopicState(
                args.output_topic
                or f'/{args.robot_name}/jparse_velocity_controller_{args.arm}/twist_cmd',
                'geometry_msgs/msg/TwistStamped',
            ),
            'raw_wrench': TopicState(
                args.wrench_topic
                or f'/{args.robot_name}/{self.arm_name}/force_torque_sensor_broadcaster/ft_data',
                'geometry_msgs/msg/WrenchStamped',
            ),
            'filtered_wrench': TopicState(
                args.filtered_wrench_topic
                or f'/{args.robot_name}/cartesian_admittance_controller_{args.arm}/filtered_wrench',
                'geometry_msgs/msg/WrenchStamped',
            ),
            'equilibrium_pose': TopicState(
                args.equilibrium_pose_topic
                or f'/{args.robot_name}/cartesian_admittance_controller_{args.arm}/equilibrium_pose',
                'geometry_msgs/msg/PoseStamped',
            ),
            'target_pose': TopicState(
                args.target_pose_topic
                or f'/{args.robot_name}/cartesian_admittance_controller_{args.arm}/target_pose',
                'geometry_msgs/msg/PoseStamped',
            ),
            'jparse_debug': TopicState(
                args.jparse_debug_topic
                or f'/{args.robot_name}/jparse_velocity_controller_{args.arm}/debug_twist',
                'std_msgs/msg/Float64MultiArray',
            ),
            'safe_joint_command': TopicState(
                args.safe_command_topic
                or f'/{args.robot_name}/{self.arm_name}/safe_forward_velocity_controller/commands',
                'std_msgs/msg/Float64MultiArray',
            ),
            'forward_joint_command': TopicState(
                args.forward_command_topic
                or f'/{args.robot_name}/{self.arm_name}/forward_velocity_controller/commands',
                'std_msgs/msg/Float64MultiArray',
            ),
            'joint_states': TopicState(
                args.joint_states_topic or '/joint_states',
                'sensor_msgs/msg/JointState',
            ),
        }

        self.create_subscription(TwistStamped, self.topics['input'].name, self._cb('input', twist_values), 10)
        self.create_subscription(
            TwistStamped,
            self.topics['admittance_output'].name,
            self._cb('admittance_output', twist_values),
            10,
        )
        self.create_subscription(
            WrenchStamped,
            self.topics['raw_wrench'].name,
            self._cb('raw_wrench', wrench_values),
            rclpy.qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            WrenchStamped,
            self.topics['filtered_wrench'].name,
            self._cb('filtered_wrench', wrench_values),
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.topics['equilibrium_pose'].name,
            self._cb('equilibrium_pose', pose_values),
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.topics['target_pose'].name,
            self._cb('target_pose', pose_values),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self.topics['jparse_debug'].name,
            self._cb('jparse_debug', multi_array_values),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self.topics['safe_joint_command'].name,
            self._cb('safe_joint_command', multi_array_values),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self.topics['forward_joint_command'].name,
            self._cb('forward_joint_command', multi_array_values),
            10,
        )
        self.create_subscription(
            JointState,
            self.topics['joint_states'].name,
            self._joint_state_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.test_pub = self.create_publisher(TwistStamped, self.topics['input'].name, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_names = [
            f'{self.arm_name}/shoulder_pan_joint',
            f'{self.arm_name}/shoulder_lift_joint',
            f'{self.arm_name}/elbow_joint',
            f'{self.arm_name}/wrist_1_joint',
            f'{self.arm_name}/wrist_2_joint',
            f'{self.arm_name}/wrist_3_joint',
        ]
        self.joint_positions = {}
        self.joint_velocities = {}
        self.candidate_wrench_topics = self._make_candidate_wrench_topics()

    def _make_log_path(self, args):
        if args.log_file == 'none':
            return None
        if args.log_file:
            return Path(args.log_file).expanduser()
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = (
            f'cartesian_admittance_{args.robot_name}_{args.arm}_{timestamp}.log'
        )
        return Path(args.log_dir).expanduser() / filename

    def _make_candidate_wrench_topics(self):
        topics = [
            self.topics['raw_wrench'].name,
            f'/{self.args.robot_name}/{self.arm_name}/force_torque_sensor_broadcaster/ft_data',
            f'/{self.args.robot_name}/{self.arm_name}/force_torque_sensor_broadcaster/wrench',
            f'/{self.args.robot_name}/{self.arm_name}/force_torque_sensor_broadcaster/wrench_stamped',
            f'/{self.args.robot_name}/{self.arm_name}/ft_data',
            f'/{self.args.robot_name}/{self.arm_name}/wrench',
            f'/{self.args.robot_name}/{self.arm_name}/wrench_stamped',
            f'/{self.args.robot_name}/{self.arm_name}/tcp_fts_sensor/ft_data',
            f'/{self.args.robot_name}/{self.arm_name}/tcp_fts_sensor/wrench',
            f'/{self.args.robot_name}/force_torque_sensor_broadcaster_{self.args.arm}/ft_data',
            f'/{self.args.robot_name}/force_torque_sensor_broadcaster_{self.args.arm}/wrench',
        ]
        unique_topics = []
        for topic in topics:
            if topic not in unique_topics:
                unique_topics.append(topic)
        return unique_topics

    def _print(self, *parts):
        text = ' '.join(str(part) for part in parts)
        print(text)
        if self.log_handle is not None:
            self.log_handle.write(text + '\n')
            self.log_handle.flush()

    def _cb(self, key, extractor):
        def callback(msg):
            self.topics[key].update(msg, time.monotonic(), extractor(msg))
        return callback

    def _joint_state_callback(self, msg):
        values = []
        for index, name in enumerate(msg.name):
            if name not in self.joint_names:
                continue
            if index < len(msg.position) and math.isfinite(msg.position[index]):
                self.joint_positions[name] = msg.position[index]
                values.append(msg.position[index])
            if index < len(msg.velocity) and math.isfinite(msg.velocity[index]):
                self.joint_velocities[name] = msg.velocity[index]
        self.topics['joint_states'].update(msg, time.monotonic(), values)

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
        self.print_graph_overview()
        start = time.monotonic()
        end = start + self.args.duration
        test_end = start + self.args.test_duration
        next_tick = start

        if self.args.send_test_command:
            self.get_logger().info(
                f'Sending test command {self.args.axis}={self.args.velocity:.4f} for '
                f'{self.args.test_duration:.2f}s on {self.topics["input"].name}'
            )

        while rclpy.ok() and time.monotonic() < end:
            now = time.monotonic()
            if self.args.send_test_command:
                self.publish_test_twist(now < test_end)
            rclpy.spin_once(self, timeout_sec=0.01)
            if now >= next_tick:
                self.print_live_tick(now)
                next_tick = now + 1.0

        if self.args.send_test_command:
            for _ in range(10):
                self.publish_test_twist(False)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.02)

        self.print_summary()
        if self.log_path is not None:
            self._print(f'\nLog file: {self.log_path}')

    def print_graph_overview(self):
        self._print('\n=== Cartesian Admittance Diagnostic ===')
        self._print(f'robot={self.args.robot_name} arm={self.args.arm}')
        self._print(f'base_frame={self.base_frame}')
        self._print(f'tcp_frame={self.tcp_frame}')
        self._print(f'command_frame={self.command_frame}')

        self._print('\n--- Topic graph ---')
        for label, state in self.topics.items():
            pubs = self.get_publishers_info_by_topic(state.name)
            subs = self.get_subscriptions_info_by_topic(state.name)
            pub_types = sorted({info.topic_type for info in pubs})
            sub_types = sorted({info.topic_type for info in subs})
            self._print(
                f'{label:22s} pubs={len(pubs):2d} subs={len(subs):2d} '
                f'{state.name} pub_types={pub_types or "-"} sub_types={sub_types or "-"}'
            )

        topic_names = self.get_topic_names_and_types()
        wrench_like = sorted(
            name for name, types in topic_names
            if 'wrench' in name.lower() or 'ft_data' in name.lower() or
            any('Wrench' in topic_type for topic_type in types)
        )
        if wrench_like:
            self._print('\nWrench-like topics found:')
            for name in wrench_like:
                self._print(f'  {name}')
        else:
            self._print('\nNo wrench-like topics found in graph.')

        self._print('\nWrench topic candidates:')
        for name in self.candidate_wrench_topics:
            pubs = self.get_publishers_info_by_topic(name)
            subs = self.get_subscriptions_info_by_topic(name)
            pub_types = sorted({info.topic_type for info in pubs})
            sub_types = sorted({info.topic_type for info in subs})
            marker = ' <-- expected' if name == self.topics['raw_wrench'].name else ''
            self._print(
                f'  pubs={len(pubs):2d} subs={len(subs):2d} {name}'
                f' pub_types={pub_types or "-"} sub_types={sub_types or "-"}{marker}'
            )

        self.print_controller_state()
        self.print_tf_check()

    def print_controller_state(self):
        service_name = f'/{self.args.robot_name}/{self.arm_name}/controller_manager/list_controllers'
        client = self.create_client(ListControllers, service_name)
        if not client.wait_for_service(timeout_sec=self.args.controller_timeout):
            self._print(f'\nController manager not reachable: {service_name}')
            return
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.controller_timeout)
        if not future.done() or future.result() is None:
            self._print(f'\nController manager did not answer: {service_name}')
            return
        self._print('\n--- Controllers ---')
        for controller in future.result().controller:
            if controller.name in (
                'forward_velocity_controller',
                'force_torque_sensor_broadcaster',
                'joint_state_broadcaster',
                'speed_scaling_state_broadcaster',
            ):
                self._print(f'  {controller.name:32s} {controller.state}')

    def print_tf_check(self):
        self._print('\n--- TF ---')
        for parent, child in (
            (self.base_frame, self.tcp_frame),
            (self.base_frame, f'{self.args.robot_name}/{self.arm_name}/admittance_equilibrium_pose'),
            (self.base_frame, f'{self.args.robot_name}/{self.arm_name}/admittance_target_pose'),
        ):
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                t = transform.transform.translation
                self._print(f'  OK {parent} -> {child}: [{t.x:.3f} {t.y:.3f} {t.z:.3f}]')
            except TransformException as exc:
                self._print(f'  MISSING {parent} -> {child}: {exc}')

    def print_live_tick(self, now):
        def brief(key):
            state = self.topics[key]
            age = state.age(now)
            if age is None:
                return f'{key}=none'
            return f'{key}=count:{state.count} age:{age:.2f}s max:{state.max_abs:.4g}'

        self._print(
            'tick '
            + ', '.join(
                brief(key)
                for key in (
                    'raw_wrench',
                    'filtered_wrench',
                    'admittance_output',
                    'jparse_debug',
                    'safe_joint_command',
                    'forward_joint_command',
                )
            )
        )

    def print_summary(self):
        now = time.monotonic()
        self._print('\n=== Summary ===')
        for label, state in self.topics.items():
            age = state.age(now)
            age_text = 'never' if age is None else f'{age:.3f}s ago'
            self._print(
                f'{label:22s} count={state.count:5d} last={age_text:>12s} '
                f'max_abs={state.max_abs:.6g} topic={state.name}'
            )
            if state.samples:
                self._print(f'  last values: {fmt_values(state.samples[-1])}')

        missing_joints = [name for name in self.joint_names if name not in self.joint_positions]
        if missing_joints:
            self._print('\nMissing joint states:')
            for name in missing_joints:
                self._print(f'  {name}')

        self._print('\n--- Interpretation hints ---')
        if self.topics['raw_wrench'].count == 0:
            self._print('* No raw wrench received. The admittance node will wait for wrench/bias and publish zero.')
            self._print('  Check the actual FT topic above and override wrench_topic if needed.')
        elif self.topics['filtered_wrench'].count == 0:
            self._print('* Raw wrench exists, but filtered_wrench is absent. The admittance node may not be running,')
            self._print('  may be waiting for TF, or may be launched with a different topic namespace.')
        elif self.topics['admittance_output'].max_abs < 1.0e-6:
            self._print('* Admittance output is zero. Possible causes: no test command, wrench bias still active,')
            self._print('  zero admittance gains, missing/failing TF, or command timeout.')
        elif self.topics['jparse_debug'].max_abs < 1.0e-6:
            self._print('* Admittance publishes nonzero twist, but J-PARSE debug stays zero.')
            self._print('  Check that J-PARSE subscribes to the same twist topic.')
        elif self.topics['safe_joint_command'].max_abs < 1.0e-6:
            self._print('* J-PARSE appears active, but safe joint command is zero.')
            self._print('  Check J-PARSE chain/joint states and singularity/debug output.')
        elif self.topics['forward_joint_command'].max_abs < 1.0e-6:
            self._print('* Safety input is nonzero, but forward controller output is zero.')
            self._print('  Check arm_velocity_safety limits/collision stop/status.')
        else:
            self._print('* Command chain produced nonzero forward joint velocity commands.')
            self._print('  If the robot still does not move, check controller state, teach pendant, safety stop,')
            self._print('  robot program running, speed slider, and UR driver logs.')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620')
    parser.add_argument('--arm', choices=['l', 'r'], default='r')
    parser.add_argument('--duration', type=float, default=6.0)
    parser.add_argument('--send-test-command', action='store_true')
    parser.add_argument('--test-duration', type=float, default=2.0)
    parser.add_argument('--axis', choices=sorted(AXIS_INDEX), default='x')
    parser.add_argument('--velocity', type=float, default=0.02)
    parser.add_argument('--base-frame')
    parser.add_argument('--tcp-frame')
    parser.add_argument('--command-frame')
    parser.add_argument('--input-topic')
    parser.add_argument('--output-topic')
    parser.add_argument('--wrench-topic')
    parser.add_argument('--filtered-wrench-topic')
    parser.add_argument('--equilibrium-pose-topic')
    parser.add_argument('--target-pose-topic')
    parser.add_argument('--jparse-debug-topic')
    parser.add_argument('--safe-command-topic')
    parser.add_argument('--forward-command-topic')
    parser.add_argument('--joint-states-topic')
    parser.add_argument('--controller-timeout', type=float, default=2.0)
    parser.add_argument('--log-dir', default='~/cartesian_admittance_diagnostics')
    parser.add_argument(
        '--log-file',
        default='',
        help="Log file path. Default: auto file in --log-dir. Use 'none' to disable.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = CartesianAdmittanceDiagnoser(args)
    try:
        node.run()
    finally:
        if node.log_handle is not None:
            node.log_handle.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
