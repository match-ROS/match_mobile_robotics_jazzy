#!/usr/bin/env python3
"""Log step responses of the integrated Cartesian admittance controller."""

import argparse
import csv
import json
import math
import os
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import TwistStamped, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


AXES = {
    "x": 0,
    "y": 1,
    "z": 2,
    "rx": 3,
    "ry": 4,
    "rz": 5,
}


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def scale(values, factor):
    return [value * factor for value in values]


def quat_normalize(q):
    q_norm = norm(q)
    if q_norm <= 1.0e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [value / q_norm for value in q]


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
    q = quat_normalize(q)
    return 2.0 * math.atan2(norm(q[:3]), abs(q[3]))


def quat_error_angle_deg(reference, current):
    return math.degrees(quat_angle(quat_multiply(current, quat_inverse(reference))))


def transform_to_pose(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (
        [translation.x, translation.y, translation.z],
        quat_normalize([rotation.x, rotation.y, rotation.z, rotation.w]),
    )


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


class StepResponseLogger(Node):
    def __init__(self, args):
        super().__init__("log_integrated_cartesian_step_response")
        self.args = args
        self.robot_name = args.robot_name
        self.arm = args.arm
        self.arm_name = f"UR10_{self.arm}"
        self.base_link = args.base_link or f"{self.arm_name}/base_link"
        self.tip_link = args.tip_link or f"{self.arm_name}/tool0"
        self.tf_base_frame = args.tf_base_frame or f"{self.robot_name}/{self.base_link}"
        self.tf_tip_frame = args.tf_tip_frame or f"{self.robot_name}/{self.tip_link}"
        self.controller_ns = (
            args.controller_ns
            or f"/{self.robot_name}/{self.arm_name}/{args.controller_name}"
        )
        self.command_topic = args.command_topic or f"{self.controller_ns}/equilibrium_twist_cmd"
        self.debug_topic = args.debug_topic or f"{self.controller_ns}/debug_twist"
        self.wrench_topic = args.filtered_wrench_topic or f"{self.controller_ns}/filtered_wrench"
        self.joint_states_topic = args.joint_states_topic or "/joint_states"
        self.joint_names = [
            f"{self.arm_name}/shoulder_pan_joint",
            f"{self.arm_name}/shoulder_lift_joint",
            f"{self.arm_name}/elbow_joint",
            f"{self.arm_name}/wrist_1_joint",
            f"{self.arm_name}/wrist_2_joint",
            f"{self.arm_name}/wrist_3_joint",
        ]

        self.publisher = self.create_publisher(TwistStamped, self.command_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_debug = []
        self.last_wrench = [0.0] * 6
        self.joint_positions = {}
        self.joint_velocities = {}

        self.create_subscription(Float64MultiArray, self.debug_topic, self.debug_callback, 10)
        self.create_subscription(WrenchStamped, self.wrench_topic, self.wrench_callback, 10)
        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

    def debug_callback(self, msg):
        self.last_debug = list(msg.data)

    def wrench_callback(self, msg):
        self.last_wrench = wrench_values(msg)

    def joint_state_callback(self, msg):
        for index, name in enumerate(msg.name):
            if name not in self.joint_names:
                continue
            if index < len(msg.position) and math.isfinite(msg.position[index]):
                self.joint_positions[name] = msg.position[index]
            if index < len(msg.velocity) and math.isfinite(msg.velocity[index]):
                self.joint_velocities[name] = msg.velocity[index]

    def run(self):
        output_dir = os.path.expanduser(self.args.output_dir)
        os.makedirs(output_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_path = os.path.join(
            output_dir,
            f"integrated_cartesian_step_response_{self.robot_name}_{self.arm}_{stamp}",
        )
        csv_path = base_path + ".csv"
        summary_path = base_path + "_summary.json"

        self.get_logger().info(f"Writing CSV log to {csv_path}")
        self.get_logger().info(f"Writing summary to {summary_path}")
        self.wait_for_controller()
        self.wait_for_start_state()

        sequence = self.make_sequence()
        summaries = []
        with open(csv_path, "w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.csv_fields())
            writer.writeheader()
            for test in sequence:
                summaries.append(self.run_segment(test, writer))
                self.zero_for(self.args.settle_duration)

        self.zero_for(0.5)
        data = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "robot_name": self.robot_name,
            "arm": self.arm,
            "command_topic": self.command_topic,
            "debug_topic": self.debug_topic,
            "filtered_wrench_topic": self.wrench_topic,
            "tf_base_frame": self.tf_base_frame,
            "tf_tip_frame": self.tf_tip_frame,
            "parameters": vars(self.args),
            "segments": summaries,
            "aggregate": self.aggregate(summaries),
        }
        with open(summary_path, "w", encoding="utf-8") as stream:
            json.dump(data, stream, indent=2)

        self.print_summary(summaries, data["aggregate"])
        self.get_logger().info(f"CSV: {csv_path}")
        self.get_logger().info(f"Summary: {summary_path}")
        return csv_path, summary_path

    def make_sequence(self):
        axis_names = [axis.strip().lower() for axis in self.args.axes.split(",") if axis.strip()]
        tests = []
        for axis_name in axis_names:
            if axis_name not in AXES:
                raise RuntimeError(f"Unknown axis '{axis_name}'. Valid: {sorted(AXES)}")
            index = AXES[axis_name]
            is_angular = index >= 3
            distance = math.radians(self.args.angular_degrees) if is_angular else self.args.linear_distance
            requested_speed = abs(self.args.angular_speed if is_angular else self.args.linear_speed)
            brake_duration = (
                self.args.angular_brake_ramp_duration
                if is_angular
                else self.args.linear_brake_ramp_duration
            )
            speed, plateau_duration, brake_duration = self.make_velocity_profile(
                abs(distance),
                requested_speed,
                brake_duration,
            )
            duration = plateau_duration + brake_duration
            command = [0.0] * 6
            command[index] = speed
            expected = abs(self.args.angular_degrees) if is_angular else abs(distance)
            tests.append({
                "name": f"+{axis_name}",
                "axis": axis_name,
                "index": index,
                "command": command,
                "duration": duration,
                "plateau_duration": plateau_duration,
                "brake_ramp_duration": brake_duration,
                "kind": "angular" if is_angular else "linear",
                "expected": expected,
            })
            if self.args.return_after_each:
                tests.append({
                    "name": f"-{axis_name}",
                    "axis": axis_name,
                    "index": index,
                    "command": [-value for value in command],
                    "duration": duration,
                    "plateau_duration": plateau_duration,
                    "brake_ramp_duration": brake_duration,
                    "kind": "angular" if is_angular else "linear",
                    "expected": expected,
                })
        return tests

    def make_velocity_profile(self, distance, speed, brake_duration):
        speed = max(abs(speed), 1.0e-9)
        brake_duration = max(0.0, brake_duration)
        if brake_duration <= 1.0e-9:
            return speed, distance / speed, 0.0

        plateau_duration = distance / speed - 0.5 * brake_duration
        if plateau_duration >= 0.0:
            return speed, plateau_duration, brake_duration

        peak_speed = 2.0 * distance / brake_duration
        return peak_speed, 0.0, brake_duration

    def run_segment(self, test, writer):
        self.get_logger().info(
            f"Segment {test['name']}: command={fmt(test['command'])}, "
            f"plateau={test['plateau_duration']:.3f}s, "
            f"brake_ramp={test['brake_ramp_duration']:.3f}s, "
            f"duration={test['duration']:.3f}s"
        )
        start_position, start_orientation = self.lookup_tool_pose()
        start_time = time.monotonic()
        command_end = start_time + test["duration"]
        settle_end = command_end + self.args.post_stop_log_duration
        rows = []
        while rclpy.ok() and time.monotonic() < settle_end:
            now = time.monotonic()
            elapsed = now - start_time
            command, phase = self.profile_command(test, elapsed)
            self.publish_twist(command)
            rclpy.spin_once(self, timeout_sec=0.0)
            row = self.capture_row(test["name"], phase, command, start_time)
            rows.append(row)
            writer.writerow(row)
            time.sleep(1.0 / self.args.sample_rate)

        self.zero_for(0.15)
        end_position, end_orientation = self.lookup_tool_pose(timeout=0.8)
        summary = self.summarize_segment(
            test,
            rows,
            start_position,
            start_orientation,
            end_position,
            end_orientation,
        )
        self.get_logger().info(
            "%s: expected=%.4f, tf_primary=%.4f, achieved_primary=%.4f, "
            "achieved_error=%.4f, lateral=%.4f, achieved_post_stop=%.4f, "
            "achieved_max_speed=%.4f, achieved_max_acc=%.4f"
            % (
                test["name"],
                summary["expected_primary"],
                summary["primary_motion"],
                summary["achieved_primary_motion"],
                summary["achieved_abs_primary_error"],
                summary["lateral_motion"],
                summary["achieved_post_stop_motion"],
                summary["achieved_max_primary_speed"],
                summary["achieved_max_primary_acceleration"],
            )
        )
        return summary

    def profile_command(self, test, elapsed):
        if elapsed < test["plateau_duration"]:
            return list(test["command"]), "command"

        brake_elapsed = elapsed - test["plateau_duration"]
        if brake_elapsed < test["brake_ramp_duration"]:
            if test["brake_ramp_duration"] <= 1.0e-9:
                return [0.0] * 6, "post_stop"
            factor = max(0.0, 1.0 - brake_elapsed / test["brake_ramp_duration"])
            return [value * factor for value in test["command"]], "brake_ramp"

        return [0.0] * 6, "post_stop"

    def summarize_segment(self, test, rows, start_position, start_orientation, end_position, end_orientation):
        index = test["index"]
        is_linear = test["kind"] == "linear"
        if is_linear:
            command_axis = [0.0, 0.0, 0.0]
            command_axis[index] = 1.0 if test["command"][index] >= 0.0 else -1.0
            final_delta = sub(end_position, start_position)
            primary = dot(final_delta, command_axis)
            lateral = norm(sub(final_delta, scale(command_axis, primary)))
            primary_values = [
                dot(sub([row["x"], row["y"], row["z"]], start_position), command_axis)
                for row in rows
            ]
            post_stop_values = [
                value for value, row in zip(primary_values, rows) if row["phase"] == "post_stop"
            ]
            orientation_change = quat_error_angle_deg(start_orientation, end_orientation)
        else:
            primary = quat_error_angle_deg(start_orientation, end_orientation)
            if test["command"][index] < 0.0:
                primary *= -1.0
            lateral = norm(sub(end_position, start_position))
            primary_values = [
                quat_error_angle_deg(start_orientation, [row["qx"], row["qy"], row["qz"], row["qw"]])
                for row in rows
            ]
            if test["command"][index] < 0.0:
                primary_values = [-value for value in primary_values]
            post_stop_values = [
                value for value, row in zip(primary_values, rows) if row["phase"] == "post_stop"
            ]
            orientation_change = abs(primary)

        expected = test["expected"]
        achieved_values, achieved_speeds, achieved_accelerations = self.integrated_achieved_values(
            rows,
            index,
            test["kind"],
        )
        achieved_primary = achieved_values[-1] if achieved_values else 0.0
        achieved_post_stop_values = [
            value for value, row in zip(achieved_values, rows) if row["phase"] == "post_stop"
        ]
        achieved_post_stop_motion = 0.0
        if achieved_post_stop_values:
            achieved_post_stop_motion = max(achieved_post_stop_values) - min(achieved_post_stop_values)
        achieved_overshoot = max([0.0] + [value - expected for value in achieved_values])
        overshoot = max([0.0] + [value - expected for value in primary_values])
        post_stop_motion = 0.0
        if post_stop_values:
            post_stop_motion = max(post_stop_values) - min(post_stop_values)

        speeds = []
        accelerations = []
        previous_value = None
        previous_speed = None
        previous_time = None
        for row, value in zip(rows, primary_values):
            current_time = row["segment_time"]
            if previous_value is not None and current_time > previous_time:
                speed = (value - previous_value) / (current_time - previous_time)
                speeds.append(speed)
                if previous_speed is not None:
                    accelerations.append((speed - previous_speed) / (current_time - previous_time))
                previous_speed = speed
            previous_value = value
            previous_time = current_time

        return {
            "name": test["name"],
            "axis": test["axis"],
            "kind": test["kind"],
            "command": test["command"],
            "duration_s": test["duration"],
            "plateau_duration_s": test["plateau_duration"],
            "brake_ramp_duration_s": test["brake_ramp_duration"],
            "samples": len(rows),
            "expected_primary": expected,
            "primary_motion": primary,
            "primary_tracking_ratio": primary / max(expected, 1.0e-9),
            "abs_primary_error": abs(expected - abs(primary)),
            "achieved_primary_motion": achieved_primary,
            "achieved_primary_tracking_ratio": achieved_primary / max(expected, 1.0e-9),
            "achieved_abs_primary_error": abs(expected - abs(achieved_primary)),
            "lateral_motion": lateral,
            "lateral_over_expected": lateral / max(expected, 1.0e-9),
            "orientation_change_deg": orientation_change,
            "primary_overshoot": overshoot,
            "post_stop_motion": post_stop_motion,
            "max_primary_speed": max([0.0] + [abs(value) for value in speeds]),
            "max_primary_acceleration": max([0.0] + [abs(value) for value in accelerations]),
            "rms_primary_acceleration": rms(accelerations),
            "achieved_primary_overshoot": achieved_overshoot,
            "achieved_post_stop_motion": achieved_post_stop_motion,
            "achieved_max_primary_speed": max([0.0] + [abs(value) for value in achieved_speeds]),
            "achieved_max_primary_acceleration": max(
                [0.0] + [abs(value) for value in achieved_accelerations]
            ),
            "achieved_rms_primary_acceleration": rms(achieved_accelerations),
            "max_abs_wrench": max_vector_norm(rows, "wrench_"),
            "max_abs_qdot": max_vector_norm(rows, "debug_qdot_"),
            "start_position": start_position,
            "end_position": end_position,
        }

    def integrated_achieved_values(self, rows, index, kind):
        values = []
        speeds = []
        accelerations = []
        integral = 0.0
        previous_time = None
        previous_speed = None
        key = [
            "debug_achieved_vx",
            "debug_achieved_vy",
            "debug_achieved_vz",
            "debug_achieved_wx",
            "debug_achieved_wy",
            "debug_achieved_wz",
        ][index]
        unit_scale = 180.0 / math.pi if kind == "angular" else 1.0

        for row in rows:
            current_time = row["segment_time"]
            speed = row.get(key, float("nan"))
            if not math.isfinite(speed):
                speed = 0.0
            speed *= unit_scale
            if previous_time is not None and current_time > previous_time:
                dt = current_time - previous_time
                integral += 0.5 * (previous_speed + speed) * dt
                accelerations.append((speed - previous_speed) / dt)
            values.append(integral)
            speeds.append(speed)
            previous_time = current_time
            previous_speed = speed
        return values, speeds, accelerations

    def aggregate(self, summaries):
        if not summaries:
            return {}
        linear = [summary for summary in summaries if summary["kind"] == "linear"]
        angular = [summary for summary in summaries if summary["kind"] == "angular"]
        return {
            "mean_abs_primary_error_linear_m": mean([s["abs_primary_error"] for s in linear]),
            "mean_achieved_abs_primary_error_linear_m": mean([
                s["achieved_abs_primary_error"] for s in linear
            ]),
            "mean_lateral_linear_m": mean([s["lateral_motion"] for s in linear]),
            "max_post_stop_motion_linear_m": max([0.0] + [s["post_stop_motion"] for s in linear]),
            "max_achieved_post_stop_motion_linear_m": max(
                [0.0] + [s["achieved_post_stop_motion"] for s in linear]
            ),
            "mean_abs_primary_error_angular_deg": mean([s["abs_primary_error"] for s in angular]),
            "mean_achieved_abs_primary_error_angular_deg": mean([
                s["achieved_abs_primary_error"] for s in angular
            ]),
            "max_post_stop_motion_angular_deg": max([0.0] + [s["post_stop_motion"] for s in angular]),
            "max_achieved_post_stop_motion_angular_deg": max(
                [0.0] + [s["achieved_post_stop_motion"] for s in angular]
            ),
            "max_primary_acceleration": max([0.0] + [s["max_primary_acceleration"] for s in summaries]),
            "max_achieved_primary_acceleration": max(
                [0.0] + [s["achieved_max_primary_acceleration"] for s in summaries]
            ),
            "max_abs_qdot": max([0.0] + [s["max_abs_qdot"] for s in summaries]),
        }

    def capture_row(self, segment, phase, command, segment_start_time):
        position, orientation = self.lookup_tool_pose(timeout=0.1)
        debug = pad(self.last_debug, 19)
        joint_positions = [self.joint_positions.get(name, float("nan")) for name in self.joint_names]
        joint_velocities = [self.joint_velocities.get(name, float("nan")) for name in self.joint_names]
        now = time.monotonic()
        row = {
            "time_monotonic": now,
            "segment_time": now - segment_start_time,
            "segment": segment,
            "phase": phase,
            "cmd_vx": command[0],
            "cmd_vy": command[1],
            "cmd_vz": command[2],
            "cmd_wx": command[3],
            "cmd_wy": command[4],
            "cmd_wz": command[5],
            "x": position[0],
            "y": position[1],
            "z": position[2],
            "qx": orientation[0],
            "qy": orientation[1],
            "qz": orientation[2],
            "qw": orientation[3],
            "debug_condition": debug[0],
            "debug_target_vx": debug[1],
            "debug_target_vy": debug[2],
            "debug_target_vz": debug[3],
            "debug_target_wx": debug[4],
            "debug_target_wy": debug[5],
            "debug_target_wz": debug[6],
            "debug_achieved_vx": debug[7],
            "debug_achieved_vy": debug[8],
            "debug_achieved_vz": debug[9],
            "debug_achieved_wx": debug[10],
            "debug_achieved_wy": debug[11],
            "debug_achieved_wz": debug[12],
        }
        for index, value in enumerate(debug[13:19]):
            row[f"debug_qdot_{index}"] = value
        for index, value in enumerate(pad(self.last_wrench, 6)):
            row[f"wrench_{index}"] = value
        for index, value in enumerate(joint_positions):
            row[f"joint_pos_{index}"] = value
        for index, value in enumerate(joint_velocities):
            row[f"joint_vel_{index}"] = value
        return row

    def csv_fields(self):
        fields = [
            "time_monotonic",
            "segment_time",
            "segment",
            "phase",
            "cmd_vx",
            "cmd_vy",
            "cmd_vz",
            "cmd_wx",
            "cmd_wy",
            "cmd_wz",
            "x",
            "y",
            "z",
            "qx",
            "qy",
            "qz",
            "qw",
            "debug_condition",
            "debug_target_vx",
            "debug_target_vy",
            "debug_target_vz",
            "debug_target_wx",
            "debug_target_wy",
            "debug_target_wz",
            "debug_achieved_vx",
            "debug_achieved_vy",
            "debug_achieved_vz",
            "debug_achieved_wx",
            "debug_achieved_wy",
            "debug_achieved_wz",
        ]
        fields.extend(f"debug_qdot_{index}" for index in range(6))
        fields.extend(f"wrench_{index}" for index in range(6))
        fields.extend(f"joint_pos_{index}" for index in range(6))
        fields.extend(f"joint_vel_{index}" for index in range(6))
        return fields

    def print_summary(self, summaries, aggregate):
        print("")
        print("Integrated Cartesian step response summary")
        for summary in summaries:
            unit = "deg" if summary["kind"] == "angular" else "m"
            print(
                "{name:>4s}: expected={expected:.4f} {unit}, tf_primary={primary:.4f}, "
                "achieved={achieved:.4f}, achieved_error={achieved_error:.4f}, "
                "lateral={lateral:.4f}, achieved_post_stop={post_stop:.4f}, "
                "achieved_max_speed={speed:.4f}, achieved_max_acc={acc:.4f}".format(
                    name=summary["name"],
                    expected=summary["expected_primary"],
                    unit=unit,
                    primary=summary["primary_motion"],
                    achieved=summary["achieved_primary_motion"],
                    achieved_error=summary["achieved_abs_primary_error"],
                    lateral=summary["lateral_motion"],
                    post_stop=summary["achieved_post_stop_motion"],
                    speed=summary["achieved_max_primary_speed"],
                    acc=summary["achieved_max_primary_acceleration"],
                )
            )
        print("")
        print("Aggregate:")
        for key, value in aggregate.items():
            print(f"  {key}: {value:.6f}")

    def wait_for_controller(self):
        deadline = time.monotonic() + self.args.discovery_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                self.get_logger().info(f"Command subscriber found on {self.command_topic}")
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn(f"No command subscriber found on {self.command_topic}; publishing anyway")
        return False

    def wait_for_start_state(self):
        deadline = time.monotonic() + self.args.discovery_timeout
        last_tf_error = "not checked yet"
        while rclpy.ok() and time.monotonic() < deadline:
            tf_ok = False
            try:
                self.lookup_tool_pose(timeout=0.2)
                tf_ok = True
            except RuntimeError as exc:
                last_tf_error = str(exc)
            missing_joints = [name for name in self.joint_names if name not in self.joint_positions]
            if tf_ok and not missing_joints:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(
            "Timed out waiting for TF and joint states. "
            f"tf={self.tf_base_frame}->{self.tf_tip_frame}, last_tf_error={last_tf_error}, "
            f"joint_states_topic={self.joint_states_topic}, missing_joints="
            f"{[name for name in self.joint_names if name not in self.joint_positions]}"
        )

    def lookup_tool_pose(self, timeout=0.5):
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
                rclpy.spin_once(self, timeout_sec=0.01)
        raise RuntimeError(
            f"Could not lookup TF {self.tf_base_frame}->{self.tf_tip_frame}: {last_error}"
        )

    def publish_twist(self, command):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_link
        msg.twist.linear.x = command[0]
        msg.twist.linear.y = command[1]
        msg.twist.linear.z = command[2]
        msg.twist.angular.x = command[3]
        msg.twist.angular.y = command[4]
        msg.twist.angular.z = command[5]
        self.publisher.publish(msg)

    def zero_for(self, duration):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_twist([0.0] * 6)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / self.args.sample_rate)


def pad(values, length):
    padded = list(values[:length])
    padded.extend([float("nan")] * max(0, length - len(padded)))
    return padded


def fmt(values):
    return "[" + " ".join(f"{value:.4f}" for value in values) + "]"


def mean(values):
    values = list(values)
    if not values:
        return 0.0
    return sum(values) / len(values)


def rms(values):
    values = list(values)
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def max_vector_norm(rows, prefix):
    result = 0.0
    for row in rows:
        values = []
        index = 0
        while f"{prefix}{index}" in row:
            value = row[f"{prefix}{index}"]
            if math.isfinite(value):
                values.append(value)
            index += 1
        result = max(result, norm(values))
    return result


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-name", default="mur620")
    parser.add_argument("--arm", default="r", choices=["l", "r"])
    parser.add_argument("--controller-name", default="integrated_cartesian_admittance_controller")
    parser.add_argument("--controller-ns", default="")
    parser.add_argument("--base-link", default="")
    parser.add_argument("--tip-link", default="")
    parser.add_argument("--tf-base-frame", default="")
    parser.add_argument("--tf-tip-frame", default="")
    parser.add_argument("--command-topic", default="")
    parser.add_argument("--debug-topic", default="")
    parser.add_argument("--filtered-wrench-topic", default="")
    parser.add_argument("--joint-states-topic", default="")
    parser.add_argument("--output-dir", default="~/integrated_cartesian_step_response_logs")
    parser.add_argument("--axes", default="x,y,z,rx,ry,rz")
    parser.add_argument("--linear-distance", type=float, default=0.10)
    parser.add_argument("--linear-speed", type=float, default=0.06)
    parser.add_argument("--linear-brake-ramp-duration", type=float, default=0.35)
    parser.add_argument("--angular-degrees", type=float, default=30.0)
    parser.add_argument("--angular-speed", type=float, default=math.radians(12.0))
    parser.add_argument("--angular-brake-ramp-duration", type=float, default=0.45)
    parser.add_argument("--sample-rate", type=float, default=100.0)
    parser.add_argument("--settle-duration", type=float, default=1.0)
    parser.add_argument("--post-stop-log-duration", type=float, default=1.0)
    parser.add_argument("--discovery-timeout", type=float, default=8.0)
    parser.add_argument("--return-after-each", action="store_true", default=True)
    parser.add_argument("--no-return-after-each", dest="return_after_each", action="store_false")
    return parser.parse_args()


def main():
    args = parse_args()
    args.sample_rate = max(5.0, args.sample_rate)
    rclpy.init()
    node = StepResponseLogger(args)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted; sending zero twist")
        node.zero_for(0.5)
    finally:
        node.zero_for(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
