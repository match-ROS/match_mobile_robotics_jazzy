#!/usr/bin/env python3
"""Log Cartesian axis response of the J-PARSE velocity controller."""

import argparse
import csv
import json
import math
import os
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def add(a, b):
    return [x + y for x, y in zip(a, b)]


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
    vector_norm = norm(q[:3])
    return 2.0 * math.atan2(vector_norm, abs(q[3]))


def quat_error_angle_deg(reference, current):
    return math.degrees(quat_angle(quat_multiply(current, quat_inverse(reference))))


def transform_to_pose(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (
        [translation.x, translation.y, translation.z],
        quat_normalize([rotation.x, rotation.y, rotation.z, rotation.w]),
    )


class AxisResponseLogger(Node):
    def __init__(self, args):
        super().__init__("log_jparse_axis_response")
        self.args = args
        self.robot_name = args.robot_name
        self.arm = args.arm
        self.base_link = args.base_link or (
            "UR10_l/base_link" if args.arm == "l" else "UR10_r/base_link"
        )
        self.tip_link = args.tip_link or (
            "UR10_l/tool0" if args.arm == "l" else "UR10_r/tool0"
        )
        self.tf_base_frame = args.tf_base_frame or f"{args.robot_name}/{self.base_link}"
        self.tf_tip_frame = args.tf_tip_frame or f"{args.robot_name}/{self.tip_link}"
        self.twist_topic = (
            args.twist_topic
            or f"/{args.robot_name}/jparse_velocity_controller_{args.arm}/twist_cmd"
        )
        self.debug_topic = (
            args.debug_topic
            or f"/{args.robot_name}/jparse_velocity_controller_{args.arm}/debug_twist"
        )
        self.safe_command_topic = (
            args.safe_command_topic
            or f"/{args.robot_name}/UR10_{args.arm}/safe_forward_velocity_controller/commands"
        )
        self.output_command_topic = (
            args.output_command_topic
            or f"/{args.robot_name}/UR10_{args.arm}/forward_velocity_controller/commands"
        )
        self.joint_states_topic = args.joint_states_topic or "/joint_states"
        self.joint_names = [
            f"UR10_{args.arm}/shoulder_pan_joint",
            f"UR10_{args.arm}/shoulder_lift_joint",
            f"UR10_{args.arm}/elbow_joint",
            f"UR10_{args.arm}/wrist_1_joint",
            f"UR10_{args.arm}/wrist_2_joint",
            f"UR10_{args.arm}/wrist_3_joint",
        ]

        self.publisher = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_debug = []
        self.last_safe_command = []
        self.last_output_command = []
        self.joint_positions = {}
        self.joint_velocities = {}

        self.create_subscription(Float64MultiArray, self.debug_topic, self.debug_callback, 10)
        self.create_subscription(
            Float64MultiArray,
            self.safe_command_topic,
            self.safe_command_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self.output_command_topic,
            self.output_command_callback,
            10,
        )
        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Using TF {self.tf_base_frame}->{self.tf_tip_frame}, "
            f"joint_states={self.joint_states_topic}"
        )

    def debug_callback(self, msg):
        self.last_debug = list(msg.data)

    def safe_command_callback(self, msg):
        self.last_safe_command = list(msg.data)

    def output_command_callback(self, msg):
        self.last_output_command = list(msg.data)

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
        base_path = os.path.join(output_dir, f"jparse_axis_response_{self.robot_name}_{self.arm}_{stamp}")
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

        self.zero_for(0.4)
        data = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "robot_name": self.robot_name,
            "arm": self.arm,
            "twist_topic": self.twist_topic,
            "debug_topic": self.debug_topic,
            "safe_command_topic": self.safe_command_topic,
            "output_command_topic": self.output_command_topic,
            "tf_base_frame": self.tf_base_frame,
            "tf_tip_frame": self.tf_tip_frame,
            "parameters": vars(self.args),
            "segments": summaries,
        }
        with open(summary_path, "w", encoding="utf-8") as stream:
            json.dump(data, stream, indent=2)

        self.print_summary(summaries)
        return csv_path, summary_path

    def make_sequence(self):
        linear_speed = abs(self.args.linear_speed)
        angular_speed = abs(self.args.angular_speed)
        linear_duration = abs(self.args.linear_distance) / max(linear_speed, 1.0e-9)
        angular_duration = abs(math.radians(self.args.angular_degrees)) / max(
            angular_speed,
            1.0e-9,
        )
        tests = []
        axes = [
            ("x", [linear_speed, 0.0, 0.0, 0.0, 0.0, 0.0], linear_duration, "linear"),
            ("y", [0.0, linear_speed, 0.0, 0.0, 0.0, 0.0], linear_duration, "linear"),
            ("z", [0.0, 0.0, linear_speed, 0.0, 0.0, 0.0], linear_duration, "linear"),
            ("rx", [0.0, 0.0, 0.0, angular_speed, 0.0, 0.0], angular_duration, "angular"),
            ("ry", [0.0, 0.0, 0.0, 0.0, angular_speed, 0.0], angular_duration, "angular"),
            ("rz", [0.0, 0.0, 0.0, 0.0, 0.0, angular_speed], angular_duration, "angular"),
        ]
        for name, command, duration, kind in axes:
            tests.append({"name": f"+{name}", "axis": name, "command": command, "duration": duration, "kind": kind})
            if self.args.return_after_each:
                tests.append({
                    "name": f"-{name}_return",
                    "axis": name,
                    "command": [-value for value in command],
                    "duration": duration,
                    "kind": kind,
                })
        return tests

    def run_segment(self, test, writer):
        self.get_logger().info(
            f"Segment {test['name']}: command={test['command']} duration={test['duration']:.3f}s"
        )
        start_position, start_orientation = self.lookup_tool_pose()
        start_time = time.monotonic()
        end_time = start_time + test["duration"]
        rows = []
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_twist(test["command"])
            rclpy.spin_once(self, timeout_sec=0.0)
            row = self.capture_row(test["name"], test["command"], start_time)
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
            "%s measured translation=[%.4f %.4f %.4f] m, primary=%.4f, "
            "lateral=%.4f, max_lateral_path=%.4f, path/chord=%.2f, orient_drift=%.2f deg"
            % (
                test["name"],
                summary["delta_translation_m"][0],
                summary["delta_translation_m"][1],
                summary["delta_translation_m"][2],
                summary["primary_translation_m"],
                summary["lateral_translation_m"],
                summary["max_lateral_path_error_m"],
                summary["path_length_over_chord"],
                summary["orientation_change_deg"],
            )
        )
        return summary

    def summarize_segment(self, test, rows, start_position, start_orientation, end_position, end_orientation):
        delta = sub(end_position, start_position)
        command = test["command"]
        linear_command = command[:3]
        angular_command = command[3:]
        if norm(linear_command) > 1.0e-12:
            axis = scale(linear_command, 1.0 / norm(linear_command))
        else:
            axis = [1.0, 0.0, 0.0]

        primary = dot(delta, axis)
        lateral_vector = sub(delta, scale(axis, primary))
        lateral = norm(lateral_vector)
        path_length = 0.0
        max_lateral_path_error = 0.0
        previous = None
        for row in rows:
            position = [row["x"], row["y"], row["z"]]
            if previous is not None:
                path_length += norm(sub(position, previous))
            previous = position
            relative = sub(position, start_position)
            projected = scale(axis, dot(relative, axis))
            max_lateral_path_error = max(
                max_lateral_path_error,
                norm(sub(relative, projected)),
            )

        chord = norm(delta)
        orientation_change = quat_error_angle_deg(start_orientation, end_orientation)
        if norm(angular_command) > 1.0e-12:
            angular_axis = scale(angular_command, 1.0 / norm(angular_command))
        else:
            angular_axis = [1.0, 0.0, 0.0]

        return {
            "name": test["name"],
            "axis": test["axis"],
            "kind": test["kind"],
            "command": command,
            "duration_s": test["duration"],
            "samples": len(rows),
            "delta_translation_m": delta,
            "primary_translation_m": primary,
            "lateral_translation_m": lateral,
            "lateral_over_abs_primary": lateral / max(abs(primary), 1.0e-9),
            "max_lateral_path_error_m": max_lateral_path_error,
            "path_length_m": path_length,
            "chord_length_m": chord,
            "path_length_over_chord": path_length / max(chord, 1.0e-9),
            "orientation_change_deg": orientation_change,
            "angular_command_axis": angular_axis,
            "start_position": start_position,
            "end_position": end_position,
        }

    def capture_row(self, segment, command, segment_start_time):
        position, orientation = self.lookup_tool_pose(timeout=0.1)
        debug = self.pad(self.last_debug, 19)
        safe_command = self.pad(self.last_safe_command, 6)
        output_command = self.pad(self.last_output_command, 6)
        joint_positions = [self.joint_positions.get(name, float("nan")) for name in self.joint_names]
        joint_velocities = [self.joint_velocities.get(name, float("nan")) for name in self.joint_names]
        row = {
            "time_monotonic": time.monotonic(),
            "segment_time": time.monotonic() - segment_start_time,
            "segment": segment,
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
        for index, value in enumerate(safe_command[:6]):
            row[f"safe_cmd_{index}"] = value
        for index, value in enumerate(output_command[:6]):
            row[f"output_cmd_{index}"] = value
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
        fields.extend(f"safe_cmd_{index}" for index in range(6))
        fields.extend(f"output_cmd_{index}" for index in range(6))
        fields.extend(f"joint_pos_{index}" for index in range(6))
        fields.extend(f"joint_vel_{index}" for index in range(6))
        return fields

    def print_summary(self, summaries):
        print("")
        print("J-PARSE axis response summary")
        for summary in summaries:
            print(
                "{name:>10s}: d=[{dx: .4f} {dy: .4f} {dz: .4f}] m, "
                "primary={primary: .4f} m, lateral={lateral:.4f} m, "
                "max_path_lateral={max_lateral:.4f} m, path/chord={ratio:.2f}, "
                "orient={orient:.2f} deg".format(
                    name=summary["name"],
                    dx=summary["delta_translation_m"][0],
                    dy=summary["delta_translation_m"][1],
                    dz=summary["delta_translation_m"][2],
                    primary=summary["primary_translation_m"],
                    lateral=summary["lateral_translation_m"],
                    max_lateral=summary["max_lateral_path_error_m"],
                    ratio=summary["path_length_over_chord"],
                    orient=summary["orientation_change_deg"],
                )
            )

    def wait_for_controller(self):
        deadline = time.monotonic() + self.args.discovery_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                self.get_logger().info(f"J-PARSE twist subscriber found on {self.twist_topic}")
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn(
            f"No J-PARSE subscriber found on {self.twist_topic}; publishing anyway"
        )
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
            missing_joints = [
                name for name in self.joint_names
                if name not in self.joint_positions
            ]
            if tf_ok and not missing_joints:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        missing_joints = [
            name for name in self.joint_names
            if name not in self.joint_positions
        ]
        raise RuntimeError(
            "Timed out waiting for TF and joint states. "
            f"tf={self.tf_base_frame}->{self.tf_tip_frame}, "
            f"last_tf_error={last_tf_error}, "
            f"joint_states_topic={self.joint_states_topic}, "
            f"missing_joints={missing_joints}"
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

    def pad(self, values, length):
        padded = list(values[:length])
        padded.extend([float("nan")] * (length - len(padded)))
        return padded


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-name", default="mur620")
    parser.add_argument("--arm", default="r", choices=["l", "r"])
    parser.add_argument("--base-link", default="")
    parser.add_argument("--tip-link", default="")
    parser.add_argument("--tf-base-frame", default="")
    parser.add_argument("--tf-tip-frame", default="")
    parser.add_argument("--twist-topic", default="")
    parser.add_argument("--debug-topic", default="")
    parser.add_argument("--safe-command-topic", default="")
    parser.add_argument("--output-command-topic", default="")
    parser.add_argument("--joint-states-topic", default="")
    parser.add_argument("--output-dir", default="~/jparse_axis_response_logs")
    parser.add_argument("--linear-distance", type=float, default=0.03)
    parser.add_argument("--linear-speed", type=float, default=0.015)
    parser.add_argument("--angular-degrees", type=float, default=5.0)
    parser.add_argument("--angular-speed", type=float, default=math.radians(3.0))
    parser.add_argument("--sample-rate", type=float, default=50.0)
    parser.add_argument("--settle-duration", type=float, default=0.8)
    parser.add_argument("--discovery-timeout", type=float, default=5.0)
    parser.add_argument("--return-after-each", action="store_true", default=True)
    parser.add_argument("--no-return-after-each", dest="return_after_each", action="store_false")
    return parser.parse_args()


def main():
    args = parse_args()
    args.sample_rate = max(5.0, args.sample_rate)
    rclpy.init()
    node = AxisResponseLogger(args)
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
