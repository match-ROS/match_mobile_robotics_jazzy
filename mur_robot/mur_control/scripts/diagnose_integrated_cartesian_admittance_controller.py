#!/usr/bin/env python3
"""Diagnose the integrated ros2_control Cartesian arm controller."""

import argparse
import json
import math
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


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
    last_text: str = ''

    def update(self, values):
        self.last_time = time.monotonic()
        self.count += 1
        finite = [abs(value) for value in values if math.isfinite(value)]
        if finite:
            self.max_abs = max(self.max_abs, max(finite))
            self.samples.append(values)
            self.samples = self.samples[-8:]

    def update_text(self, text):
        self.last_time = time.monotonic()
        self.count += 1
        self.last_text = str(text)
        self.samples.append(self.last_text)
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


def float_value(msg):
    return [msg.data]


def fmt(values):
    if isinstance(values, str):
        return values
    return '[' + ' '.join(f'{value:.4f}' for value in values) + ']'


def vector_norm(values):
    return math.sqrt(sum(value * value for value in values if math.isfinite(value)))


def diff_norm(a, b):
    return vector_norm([left - right for left, right in zip(a, b)])


def direction_cosine(a, b):
    norm_a = vector_norm(a)
    norm_b = vector_norm(b)
    if norm_a <= 1.0e-12 or norm_b <= 1.0e-12:
        return None
    return sum(left * right for left, right in zip(a, b)) / (norm_a * norm_b)


def limit_norm(values, limit):
    magnitude = vector_norm(values)
    if limit <= 0.0 or magnitude <= limit or magnitude < 1.0e-12:
        return values
    return [value * limit / magnitude for value in values]


def stage_name(value):
    stages = {
        0: 'none',
        1: 'velocity',
        2: 'acceleration',
        3: 'jerk',
        4: 'position',
        5: 'zero',
    }
    try:
        return stages.get(int(round(value)), f'unknown({value:.0f})')
    except (TypeError, ValueError):
        return 'unknown'


class IntegratedCartesianDiagnoser(Node):
    def __init__(self, args):
        super().__init__('diagnose_integrated_cartesian_admittance_controller')
        self.args = args
        self.robot_name = args.robot_name
        self.arm = args.arm
        self.arm_name = f'UR10_{self.arm}'
        self.other_arm = 'l' if self.arm == 'r' else 'r'
        self.other_arm_name = f'UR10_{self.other_arm}'
        self.controller_name = args.controller_name
        self.controller_ns = f'/{self.robot_name}/{self.arm_name}/{self.controller_name}'
        self.command_frame = args.command_frame or f'{self.arm_name}/base_link'
        self.log_handle = None
        self.log_path = self._make_log_path()
        if self.log_path is not None:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log_handle = self.log_path.open('w', encoding='utf-8')
            self._print(f'Logging diagnostic output to {self.log_path}')
        self.json_path = self._make_json_path()

        self.topics = {
            'input': TopicState(args.input_topic or f'{self.controller_ns}/equilibrium_twist_cmd'),
            'debug_twist': TopicState(args.debug_topic or f'{self.controller_ns}/debug_twist'),
            'singular_values': TopicState(args.singular_topic or f'{self.controller_ns}/singular_values'),
            'filtered_wrench': TopicState(args.filtered_wrench_topic or f'{self.controller_ns}/filtered_wrench'),
            'equilibrium_pose': TopicState(args.equilibrium_pose_topic or f'{self.controller_ns}/equilibrium_pose'),
            'target_pose': TopicState(args.target_pose_topic or f'{self.controller_ns}/target_pose'),
            'collision_min_clearance': TopicState(
                args.collision_min_clearance_topic or f'{self.controller_ns}/collision_min_clearance'
            ),
            'collision_nearest_source': TopicState(
                args.collision_nearest_source_topic
                or f'{self.controller_ns}/collision_nearest_source'
            ),
            'collision_status': TopicState(
                args.collision_status_topic or f'{self.controller_ns}/collision_status'
            ),
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
        self.other_joint_names = [
            f'{self.other_arm_name}/shoulder_pan_joint',
            f'{self.other_arm_name}/shoulder_lift_joint',
            f'{self.other_arm_name}/elbow_joint',
            f'{self.other_arm_name}/wrist_1_joint',
            f'{self.other_arm_name}/wrist_2_joint',
            f'{self.other_arm_name}/wrist_3_joint',
        ]
        self.joint_positions = {}
        self.other_joint_positions = {}
        self.controller_parameters = {}

        self.create_subscription(TwistStamped, self.topics['input'].name, self._cb('input', twist_values), 10)
        self.create_subscription(Float64MultiArray, self.topics['debug_twist'].name, self._cb('debug_twist', array_values), 10)
        self.create_subscription(Float64MultiArray, self.topics['singular_values'].name, self._cb('singular_values', array_values), 10)
        self.create_subscription(WrenchStamped, self.topics['filtered_wrench'].name, self._cb('filtered_wrench', wrench_values), 10)
        self.create_subscription(PoseStamped, self.topics['equilibrium_pose'].name, self._cb('equilibrium_pose', pose_values), 10)
        self.create_subscription(PoseStamped, self.topics['target_pose'].name, self._cb('target_pose', pose_values), 10)
        self.create_subscription(
            Float64,
            self.topics['collision_min_clearance'].name,
            self._cb('collision_min_clearance', float_value),
            10,
        )
        self.create_subscription(
            String,
            self.topics['collision_nearest_source'].name,
            self._string_cb('collision_nearest_source'),
            10,
        )
        self.create_subscription(
            String,
            self.topics['collision_status'].name,
            self._string_cb('collision_status'),
            10,
        )
        self.create_subscription(JointState, self.topics['joint_states'].name, self._joint_cb, rclpy.qos.qos_profile_sensor_data)
        self.test_pub = self.create_publisher(TwistStamped, self.topics['input'].name, 10)
        self.next_test_publish_time = 0.0

    def _make_log_path(self):
        if self.args.log_file == 'none':
            return None
        if self.args.log_file:
            return Path(self.args.log_file).expanduser()
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        name = f'integrated_cartesian_admittance_{self.args.robot_name}_{self.args.arm}_{stamp}.log'
        return Path(self.args.log_dir).expanduser() / name

    def _make_json_path(self):
        if self.log_path is None:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            name = f'integrated_cartesian_admittance_{self.args.robot_name}_{self.args.arm}_{stamp}.json'
            return Path(self.args.log_dir).expanduser() / name
        return self.log_path.with_suffix('.json')

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

    def _string_cb(self, key):
        def callback(msg):
            self.topics[key].update_text(msg.data)
        return callback

    def _joint_cb(self, msg):
        values = []
        for index, name in enumerate(msg.name):
            if name not in self.joint_names and name not in self.other_joint_names:
                continue
            if index < len(msg.position) and math.isfinite(msg.position[index]):
                if name in self.joint_names:
                    self.joint_positions[name] = msg.position[index]
                if name in self.other_joint_names:
                    self.other_joint_positions[name] = msg.position[index]
                values.append(msg.position[index])
        self.topics['joint_states'].update(values)

    def test_scale(self, now, start):
        elapsed = now - start
        ramp = max(0.0, self.args.test_ramp_duration)
        if elapsed < 0.0 or elapsed > self.args.test_duration:
            return 0.0
        scale = 1.0
        if ramp > 1.0e-6:
            scale = min(scale, elapsed / ramp)
            scale = min(scale, max(0.0, (self.args.test_duration - elapsed) / ramp))
        return max(0.0, min(1.0, scale))

    def publish_test_twist(self, scale):
        values = [0.0] * 6
        values[AXIS_INDEX[self.args.axis]] = self.args.velocity * scale
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
        self.read_controller_parameters()
        start = time.monotonic()
        end = start + self.args.duration
        next_tick = start
        test_period = 1.0 / max(1.0, self.args.test_publish_rate_hz)
        self.next_test_publish_time = start
        if self.args.send_test_command:
            self.get_logger().info(
                f'Sending rate-limited test twist {self.args.axis}={self.args.velocity:.4f} '
                f'for {self.args.test_duration:.2f}s at {self.args.test_publish_rate_hz:.1f} Hz '
                f'with {self.args.test_ramp_duration:.2f}s ramps to {self.topics["input"].name}'
            )
        while rclpy.ok() and time.monotonic() < end:
            now = time.monotonic()
            if self.args.send_test_command and now >= self.next_test_publish_time:
                self.publish_test_twist(self.test_scale(now, start))
                self.next_test_publish_time += test_period
                if self.next_test_publish_time < now - test_period:
                    self.next_test_publish_time = now + test_period
            rclpy.spin_once(self, timeout_sec=0.01)
            if now >= next_tick:
                self.print_tick()
                next_tick = now + 1.0
        if self.args.send_test_command:
            for _ in range(10):
                self.publish_test_twist(0.0)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(test_period)
        self.print_summary()
        self.write_json_summary()
        if self.log_path is not None:
            self._print(f'\nLog file: {self.log_path}')
        self._print(f'JSON file: {self.json_path}')

    def print_overview(self):
        self._print('\n=== Integrated Cartesian Admittance Controller Diagnostic ===')
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

    def read_controller_parameters(self):
        service = f'{self.controller_ns}/get_parameters'
        client = self.create_client(GetParameters, service)
        names = [
            'use_ft_sensor',
            'require_wrench',
            'wrench_in_tcp_frame',
            'reset_equilibrium_on_zero_command',
            'admittance',
            'wrench_twist_gain',
            'pose_error_gain',
            'inverse_mode',
            'gamma',
            'damping',
            'singular_gain_position',
            'singular_gain_angular',
            'max_linear_velocity',
            'max_angular_velocity',
            'command_timeout',
            'enable_collision_avoidance',
            'collision_common_link',
            'collision_other_prefix',
            'collision_other_base_link',
            'collision_other_tip_link',
            'collision_own_base_xyz',
            'collision_own_base_rpy',
            'collision_other_base_xyz',
            'collision_other_base_rpy',
            'collision_sample_spacing',
            'collision_sphere_radius',
            'collision_activation_clearance',
            'collision_stop_clearance',
            'collision_response_mode',
            'collision_joint_state_timeout',
            'collision_fail_safe_stop',
        ]
        if not client.wait_for_service(timeout_sec=self.args.controller_timeout):
            self._print(f'\nParameter service not reachable: {service}')
            return
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.controller_timeout)
        if not future.done() or future.result() is None:
            self._print(f'\nParameter service did not answer: {service}')
            return
        self._print('\n--- Controller Parameters ---')
        for name, value in zip(names, future.result().values):
            parsed = self.parameter_value_to_python(value)
            self.controller_parameters[name] = parsed
            self._print(f'  {name}: {parsed}')

    @staticmethod
    def parameter_value_to_python(value):
        if value.type == ParameterType.PARAMETER_BOOL:
            return value.bool_value
        if value.type == ParameterType.PARAMETER_INTEGER:
            return value.integer_value
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return value.double_value
        if value.type == ParameterType.PARAMETER_STRING:
            return value.string_value
        if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(value.double_array_value)
        if value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return list(value.string_array_value)
        if value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(value.bool_array_value)
        if value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(value.integer_array_value)
        return None

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
                for key in (
                    'input',
                    'debug_twist',
                    'singular_values',
                    'filtered_wrench',
                    'collision_status',
                    'collision_nearest_source',
                    'collision_min_clearance',
                    'joint_states',
                )
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
            self._print('\nMissing own arm joint states:')
            for name in missing:
                self._print(f'  {name}')
        other_missing = [name for name in self.other_joint_names if name not in self.other_joint_positions]
        if other_missing:
            self._print('\nMissing other arm joint states:')
            for name in other_missing:
                self._print(f'  {name}')
        if not missing:
            self._print('\nOwn arm joint positions:')
            for name in self.joint_names:
                self._print(f'  {name}: {self.joint_positions[name]: .6f}')
        if not other_missing:
            self._print('\nOther arm joint positions:')
            for name in self.other_joint_names:
                self._print(f'  {name}: {self.other_joint_positions[name]: .6f}')
        self._print('\n--- Interpretation hints ---')
        if self.topics['debug_twist'].count == 0:
            self._print('* No debug_twist received. Check whether integrated_cartesian_admittance_controller is active.')
        elif self.args.send_test_command and self.topics['debug_twist'].max_abs < 1.0e-6:
            self._print('* Test command was sent, but debug output stayed zero. Check command topic and controller state.')
        elif self.args.send_test_command:
            self._print('* Integrated controller produced nonzero debug output for the test command.')
        if self.topics['joint_states'].count == 0:
            self._print('* No joint_states received; the controller cannot compute FK/Jacobian without state interfaces.')
        if self.topics['collision_status'].count == 0:
            self._print('* No collision_status received. Check whether collision avoidance is enabled and the controller is configured.')
        elif self.topics['collision_status'].last_text == 'stale_other_arm':
            self._print('* Collision status is stale_other_arm: start both arms or disable collision avoidance for one-arm tests.')
        if self.topics['collision_nearest_source'].last_text:
            self._print(
                f'* Nearest collision source: {self.topics["collision_nearest_source"].last_text}'
            )
        if self.topics['collision_min_clearance'].samples:
            clearance = self.topics['collision_min_clearance'].samples[-1][0]
            if clearance < 0.0:
                self._print(
                    '* Collision clearance is negative. If the arms are visually separated, suspect wrong '
                    'mount transforms or sampled base/link points that overlap near the robot center.'
                )
        self.print_debug_interpretation()

    def print_debug_interpretation(self):
        debug = self.topics['debug_twist'].samples[-1] if self.topics['debug_twist'].samples else []
        if len(debug) < 21:
            return
        self._print('\n--- Latest debug_twist decoded ---')
        target_twist = debug[1:7]
        achieved_twist = debug[7:13]
        target_linear_norm = vector_norm(target_twist[:3])
        target_angular_norm = vector_norm(target_twist[3:])
        achieved_linear_norm = vector_norm(achieved_twist[:3])
        achieved_angular_norm = vector_norm(achieved_twist[3:])
        self._print(
            f'  target_twist:   {fmt(target_twist)} '
            f'(lin={target_linear_norm:.4f}, ang={target_angular_norm:.4f})'
        )
        self._print(
            f'  achieved_twist: {fmt(achieved_twist)} '
            f'(lin={achieved_linear_norm:.4f}, ang={achieved_angular_norm:.4f})'
        )
        if len(debug) >= 15:
            source = self.topics['collision_nearest_source'].last_text or 'unknown'
            self._print(
                f'  collision: min_clearance={debug[13]:.4f}, scale={debug[14]:.4f}, '
                f'status={self.topics["collision_status"].last_text or "unknown"}, '
                f'source={source}'
            )
        if len(debug) >= 38:
            safety = {
                'velocity_scale': debug[13 + 2],
                'acceleration_scale': debug[13 + 3],
                'jerk_scale': debug[13 + 4],
                'limiting_joint_index': debug[13 + 5],
                'limiting_stage': debug[13 + 6],
            }
            qdot_raw = debug[20:26]
            qdot_collision = debug[26:32]
            qdot_safety = debug[32:38]
            joint_index = int(round(safety['limiting_joint_index']))
            joint_name = (
                self.joint_names[joint_index]
                if 0 <= joint_index < len(self.joint_names)
                else 'none'
            )
            self._print(
                f"  safety scales: vel={safety['velocity_scale']:.4f}, "
                f"acc={safety['acceleration_scale']:.4f}, jerk={safety['jerk_scale']:.4f}, "
                f"limiting={joint_name}, stage={stage_name(safety['limiting_stage'])}"
            )
            self._print(f'  qdot raw:       {fmt(qdot_raw)}')
            self._print(f'  qdot collision: {fmt(qdot_collision)}')
            self._print(f'  qdot safety:    {fmt(qdot_safety)}')
            collision_delta = diff_norm(qdot_raw, qdot_collision)
            safety_delta = diff_norm(qdot_collision, qdot_safety)
            collision_direction = direction_cosine(qdot_raw, qdot_collision)
            self._print(
                f'  qdot deltas: collision={collision_delta:.6f}, safety={safety_delta:.6f}'
            )
            if collision_direction is not None:
                self._print(
                    f'  qdot collision direction cosine={collision_direction:.4f}'
                )
        else:
            qdot_offset = 15 if len(debug) >= 21 else 13
            self._print(f'  legacy qdot: {fmt(debug[qdot_offset:qdot_offset + 6])}')

        threshold = self.args.angular_warning_threshold
        latest_input = self.topics['input'].samples[-1] if self.topics['input'].samples else None
        input_angular_norm = vector_norm(latest_input[3:]) if latest_input else 0.0
        input_linear_norm = vector_norm(latest_input[:3]) if latest_input else 0.0
        self._print('\n--- Motion coupling checks ---')
        if latest_input is not None:
            self._print(
                f'  latest_input:   {fmt(latest_input)} '
                f'(lin={input_linear_norm:.4f}, ang={input_angular_norm:.4f})'
            )
        if latest_input is None and (target_linear_norm > threshold or target_angular_norm > threshold):
            self._print(
                '* Controller is producing a nonzero target_twist without a fresh input command. '
                'For jog tests this usually means the equilibrium pose is stale or admittance/force '
                'feedback is still pulling the TCP.'
            )
        if input_angular_norm <= threshold and target_angular_norm > threshold:
            self._print(
                '* Angular target_twist is present although the latest input has no angular command. '
                'This comes from pose-error/admittance feedback inside the controller, not from the GUI button.'
            )
        if target_angular_norm <= threshold and achieved_angular_norm > threshold:
            self._print(
                '* Target angular twist is near zero but achieved angular twist is not. '
                'That points to inverse kinematics, collision projection, or safety limiting coupling.'
            )
        if self.topics['collision_status'].last_text == 'limited':
            self._print(
                '* Collision avoidance is actively limiting this arm. With '
                'collision_response_mode=scale qdot_collision should stay parallel to '
                'qdot_raw; '
                'if the direction cosine is far below '
                '1.0, inspect response mode, collision geometry, and RViz markers.'
            )

    def write_json_summary(self):
        self.json_path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            'created_at': datetime.now().isoformat(timespec='seconds'),
            'robot_name': self.robot_name,
            'arm': self.arm,
            'controller_ns': self.controller_ns,
            'topics': {
                key: {
                    'name': state.name,
                    'count': state.count,
                    'age': state.age(),
                    'max_abs': state.max_abs,
                    'last_text': state.last_text,
                    'last_sample': state.samples[-1] if state.samples else None,
                }
                for key, state in self.topics.items()
            },
            'controller_parameters': self.controller_parameters,
            'own_joint_positions': self.joint_positions,
            'other_joint_positions': self.other_joint_positions,
            'args': vars(self.args),
        }
        with self.json_path.open('w', encoding='utf-8') as stream:
            json.dump(data, stream, indent=2)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620')
    parser.add_argument('--arm', choices=['l', 'r'], default='r')
    parser.add_argument('--controller-name', default='integrated_cartesian_admittance_controller')
    parser.add_argument('--duration', type=float, default=6.0)
    parser.add_argument('--send-test-command', action='store_true')
    parser.add_argument('--test-duration', type=float, default=2.0)
    parser.add_argument('--test-publish-rate-hz', type=float, default=50.0)
    parser.add_argument('--test-ramp-duration', type=float, default=0.4)
    parser.add_argument('--axis', choices=sorted(AXIS_INDEX), default='x')
    parser.add_argument('--velocity', type=float, default=0.01)
    parser.add_argument('--command-frame')
    parser.add_argument('--input-topic')
    parser.add_argument('--debug-topic')
    parser.add_argument('--singular-topic')
    parser.add_argument('--filtered-wrench-topic')
    parser.add_argument('--equilibrium-pose-topic')
    parser.add_argument('--target-pose-topic')
    parser.add_argument('--collision-status-topic')
    parser.add_argument('--collision-nearest-source-topic')
    parser.add_argument('--collision-min-clearance-topic')
    parser.add_argument('--joint-states-topic')
    parser.add_argument('--controller-timeout', type=float, default=2.0)
    parser.add_argument('--angular-warning-threshold', type=float, default=0.02)
    parser.add_argument('--log-dir', default='~/integrated_cartesian_admittance_diagnostics')
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
