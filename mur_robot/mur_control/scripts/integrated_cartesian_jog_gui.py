#!/usr/bin/env python3
"""OpenCV jog GUI for the integrated Cartesian admittance controller."""

import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformException, TransformListener


FIELD_NAMES = ("x", "y", "z", "rx", "ry", "rz")


class IntegratedCartesianJogGui(Node):
    def __init__(self):
        super().__init__("integrated_cartesian_jog_gui")

        self.declare_parameter("robot_name", "mur620")
        self.declare_parameter("arm", "r")
        self.declare_parameter("controller_name", "integrated_cartesian_admittance_controller")
        self.declare_parameter("twist_topic", "")
        self.declare_parameter("tf_base_frame", "")
        self.declare_parameter("tf_tip_frame", "")
        self.declare_parameter("command_frame", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("linear_velocity", 0.03)
        self.declare_parameter("angular_velocity", 0.25)
        self.declare_parameter("linear_acceleration", 0.12)
        self.declare_parameter("angular_acceleration", 1.0)
        self.declare_parameter("hold_timeout", 0.45)
        self.declare_parameter("target_max_linear_velocity", 0.04)
        self.declare_parameter("target_max_angular_velocity", 0.20)
        self.declare_parameter("target_position_gain", 0.8)
        self.declare_parameter("target_orientation_gain", 0.8)
        self.declare_parameter("target_position_tolerance", 0.003)
        self.declare_parameter("target_orientation_tolerance_deg", 1.0)
        self.declare_parameter("target_timeout", 30.0)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("window_name", "mur620 integrated cartesian jog")
        self.declare_parameter("log_key_codes", False)

        self.robot_name = str(self.get_parameter("robot_name").value)
        self.arm = str(self.get_parameter("arm").value)
        self.arm_name = f"UR10_{self.arm}"
        self.controller_name = str(self.get_parameter("controller_name").value)
        self.controller_ns = f"/{self.robot_name}/{self.arm_name}/{self.controller_name}"
        self.twist_topic = self.param_or_default(
            "twist_topic", f"{self.controller_ns}/equilibrium_twist_cmd"
        )
        self.tf_base_frame = self.param_or_default(
            "tf_base_frame", f"{self.robot_name}/{self.arm_name}/base_link"
        )
        self.tf_tip_frame = self.param_or_default(
            "tf_tip_frame", f"{self.robot_name}/{self.arm_name}/tool0"
        )
        self.command_frame = self.param_or_default("command_frame", f"{self.arm_name}/base_link")
        self.window_name = str(self.get_parameter("window_name").value)

        self.publisher = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joint_names = [
            f"{self.arm_name}/shoulder_pan_joint",
            f"{self.arm_name}/shoulder_lift_joint",
            f"{self.arm_name}/elbow_joint",
            f"{self.arm_name}/wrist_1_joint",
            f"{self.arm_name}/wrist_2_joint",
            f"{self.arm_name}/wrist_3_joint",
        ]
        self.joint_positions = {}
        self.joint_stamp_monotonic = 0.0
        self.current_pose = None
        self.current_pose_stamp_monotonic = 0.0
        self.equilibrium_pose = None
        self.target_pose_topic_pose = None
        self.collision_status = "n/a"
        self.collision_clearance = None
        self.last_tf_error = ""

        self.target_values = np.zeros(6, dtype=np.float64)
        self.target_fields = {name: "0.0000" for name in FIELD_NAMES}
        self.target_initialized = False
        self.active_field = 0
        self.target_running = False
        self.target_started = 0.0
        self.target_linear_error = 0.0
        self.target_angular_error = 0.0

        self.target_linear = np.zeros(3, dtype=np.float64)
        self.target_angular = np.zeros(3, dtype=np.float64)
        self.current_linear = np.zeros(3, dtype=np.float64)
        self.current_angular = np.zeros(3, dtype=np.float64)
        self.last_command_time = 0.0
        self.last_update_time = time.monotonic()
        self.rotation_mode = False
        self.last_key_text = "none"
        self.last_command_text = "idle"
        self.mouse_button_command = None
        self.buttons = []
        self.field_rects = []

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self.joint_state_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped, f"{self.controller_ns}/equilibrium_pose", self.equilibrium_pose_callback, 10
        )
        self.create_subscription(
            PoseStamped, f"{self.controller_ns}/target_pose", self.target_pose_callback, 10
        )
        self.create_subscription(
            String, f"{self.controller_ns}/collision_status", self.collision_status_callback, 10
        )
        self.create_subscription(
            Float64,
            f"{self.controller_ns}/collision_min_clearance",
            self.collision_clearance_callback,
            10,
        )

        self.get_logger().info(
            f"Integrated Cartesian jog GUI publishing {self.twist_topic} in frame {self.command_frame}"
        )

    def run(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.on_mouse)
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                self.update_current_pose()
                self.update_output()
                cv2.imshow(self.window_name, self.render())
                key = cv2.waitKeyEx(20)
                if key != -1 and not self.handle_key(key):
                    break
        finally:
            self.stop()
            cv2.destroyWindow(self.window_name)

    def render(self):
        image = np.full((760, 1040, 3), 32, dtype=np.uint8)
        self.buttons = []
        self.field_rects = []

        self.put_text(image, "Integrated Cartesian Jog", (24, 42), scale=1.0, thickness=2)
        self.put_text(image, f"topic: {self.twist_topic}", (24, 78), scale=0.55)
        self.put_text(image, f"tf: {self.tf_base_frame} -> {self.tf_tip_frame}", (24, 104), scale=0.55)
        self.put_text(image, f"cmd frame: {self.command_frame}", (24, 130), scale=0.55)
        self.put_text(
            image,
            f"mode: {'rotation' if self.rotation_mode else 'translation'} | last key: {self.last_key_text}",
            (24, 160),
            scale=0.58,
            color=(190, 230, 255),
        )
        self.put_text(image, f"cmd: {self.last_command_text}", (24, 188), scale=0.56)
        sub_color = (180, 235, 180) if self.publisher.get_subscription_count() > 0 else (120, 140, 255)
        self.put_text(
            image,
            f"subscribers: {self.publisher.get_subscription_count()}",
            (760, 78),
            scale=0.58,
            color=sub_color,
        )
        clearance = "n/a" if self.collision_clearance is None else f"{self.collision_clearance:.3f} m"
        self.put_text(
            image,
            f"collision: {self.collision_status}  clearance: {clearance}",
            (760, 106),
            scale=0.55,
            color=(220, 220, 170),
        )

        self.draw_pose_panel(image, (24, 225))
        self.draw_joint_panel(image, (520, 225))
        self.draw_jog_buttons(image, (24, 510))
        self.draw_target_panel(image, (520, 500))

        self.put_text(
            image,
            "Keys: arrows XY, PgUp/PgDn Z, m rotation, c copy pose, g go target, tab field, space stop, q quit",
            (24, 735),
            scale=0.52,
            color=(220, 220, 220),
        )
        return image

    def draw_pose_panel(self, image, origin):
        x0, y0 = origin
        self.put_text(image, "TCP pose", (x0, y0), scale=0.72, color=(255, 255, 255), thickness=2)
        if self.current_pose is None:
            self.put_text(image, f"TF missing: {self.last_tf_error[:58]}", (x0, y0 + 34), color=(140, 160, 255))
            return
        position, quat = self.current_pose
        rpy = np.degrees(quat_to_rpy(quat))
        self.put_text(image, f"x {position[0]: .4f}  y {position[1]: .4f}  z {position[2]: .4f} m", (x0, y0 + 34))
        self.put_text(image, f"rx {rpy[0]: .2f}  ry {rpy[1]: .2f}  rz {rpy[2]: .2f} deg", (x0, y0 + 64))
        self.put_text(
            image,
            f"age {time.monotonic() - self.current_pose_stamp_monotonic: .2f}s",
            (x0, y0 + 94),
            color=(190, 230, 190),
        )
        if self.equilibrium_pose is not None:
            eq_pos, eq_quat = self.equilibrium_pose
            eq_rpy = np.degrees(quat_to_rpy(eq_quat))
            self.put_text(image, "equilibrium", (x0, y0 + 136), scale=0.62, color=(200, 220, 255))
            self.put_text(image, f"p [{eq_pos[0]:.3f}, {eq_pos[1]:.3f}, {eq_pos[2]:.3f}]", (x0, y0 + 164), scale=0.56)
            self.put_text(image, f"rpy [{eq_rpy[0]:.1f}, {eq_rpy[1]:.1f}, {eq_rpy[2]:.1f}]", (x0, y0 + 190), scale=0.56)
        if self.target_pose_topic_pose is not None:
            tgt_pos, tgt_quat = self.target_pose_topic_pose
            tgt_rpy = np.degrees(quat_to_rpy(tgt_quat))
            self.put_text(image, "controller target", (x0, y0 + 226), scale=0.62, color=(220, 220, 170))
            self.put_text(image, f"p [{tgt_pos[0]:.3f}, {tgt_pos[1]:.3f}, {tgt_pos[2]:.3f}]", (x0, y0 + 254), scale=0.56)
            self.put_text(image, f"rpy [{tgt_rpy[0]:.1f}, {tgt_rpy[1]:.1f}, {tgt_rpy[2]:.1f}]", (x0, y0 + 280), scale=0.56)

    def draw_joint_panel(self, image, origin):
        x0, y0 = origin
        age = time.monotonic() - self.joint_stamp_monotonic if self.joint_stamp_monotonic else None
        age_text = "never" if age is None else f"{age:.2f}s"
        self.put_text(image, f"Joint positions  age {age_text}", (x0, y0), scale=0.72, thickness=2)
        for i, name in enumerate(self.joint_names):
            short = name.split("/", 1)[1].replace("_joint", "")
            y = y0 + 34 + i * 30
            if name in self.joint_positions:
                value = self.joint_positions[name]
                self.put_text(image, f"{short:14s} {value: .4f} rad  {math.degrees(value): .1f} deg", (x0, y))
            else:
                self.put_text(image, f"{short:14s} missing", (x0, y), color=(140, 160, 255))

    def draw_jog_buttons(self, image, origin):
        x0, y0 = origin
        self.put_text(image, "Jog", (x0, y0 - 20), scale=0.72, thickness=2)
        self.add_button(image, "Y+", (x0 + 120, y0 + 0, 100, 52), [0, 1, 0], [0, 0, 0])
        self.add_button(image, "Y-", (x0 + 120, y0 + 116, 100, 52), [0, -1, 0], [0, 0, 0])
        self.add_button(image, "X-", (x0 + 12, y0 + 58, 100, 52), [-1, 0, 0], [0, 0, 0])
        self.add_button(image, "X+", (x0 + 228, y0 + 58, 100, 52), [1, 0, 0], [0, 0, 0])
        self.add_button(image, "Z+", (x0 + 360, y0 + 0, 100, 52), [0, 0, 1], [0, 0, 0])
        self.add_button(image, "Z-", (x0 + 360, y0 + 116, 100, 52), [0, 0, -1], [0, 0, 0])
        self.add_button(image, "RX+", (x0 + 12, y0 + 190, 70, 46), [0, 0, 0], [1, 0, 0])
        self.add_button(image, "RX-", (x0 + 88, y0 + 190, 70, 46), [0, 0, 0], [-1, 0, 0])
        self.add_button(image, "RY+", (x0 + 174, y0 + 190, 70, 46), [0, 0, 0], [0, 1, 0])
        self.add_button(image, "RY-", (x0 + 250, y0 + 190, 70, 46), [0, 0, 0], [0, -1, 0])
        self.add_button(image, "RZ+", (x0 + 336, y0 + 190, 70, 46), [0, 0, 0], [0, 0, 1])
        self.add_button(image, "RZ-", (x0 + 412, y0 + 190, 70, 46), [0, 0, 0], [0, 0, -1])
        self.add_button(image, "STOP", (x0 + 155, y0 + 250, 170, 44), [0, 0, 0], [0, 0, 0])

    def draw_target_panel(self, image, origin):
        x0, y0 = origin
        self.put_text(image, "Target pose in arm base", (x0, y0 - 20), scale=0.72, thickness=2)
        labels = ("x m", "y m", "z m", "rx deg", "ry deg", "rz deg")
        for i, (name, label) in enumerate(zip(FIELD_NAMES, labels)):
            row = i // 3
            col = i % 3
            x = x0 + col * 165
            y = y0 + row * 78
            self.put_text(image, label, (x, y), scale=0.55, color=(210, 210, 210))
            rect = (x, y + 10, 140, 42)
            self.field_rects.append((name, rect))
            color = (80, 105, 135) if i == self.active_field else (56, 56, 56)
            cv2.rectangle(image, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), color, -1)
            cv2.rectangle(image, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (170, 170, 170), 1)
            self.put_text(image, self.target_fields[name], (rect[0] + 8, rect[1] + 29), scale=0.62)
        self.add_action_button(image, "COPY C", (x0, y0 + 170, 130, 48), "copy")
        self.add_action_button(image, "GO G", (x0 + 150, y0 + 170, 130, 48), "go")
        self.add_action_button(image, "STOP S", (x0 + 300, y0 + 170, 130, 48), "stop")
        status = "target: idle"
        if self.target_running:
            status = (
                f"target: running  pos_err={self.target_linear_error:.4f} m  "
                f"rot_err={math.degrees(self.target_angular_error):.2f} deg"
            )
        self.put_text(image, status, (x0, y0 + 248), scale=0.56, color=(220, 220, 170))

    def add_button(self, image, label, rect, linear_unit, angular_unit):
        self.buttons.append({
            "rect": rect,
            "linear": np.asarray(linear_unit, dtype=np.float64),
            "angular": np.asarray(angular_unit, dtype=np.float64),
            "label": label,
            "action": "jog",
        })
        active = self.mouse_button_command == label
        color = (72, 116, 154) if active else (62, 62, 62)
        self.draw_labeled_rect(image, rect, label, color)

    def add_action_button(self, image, label, rect, action):
        self.buttons.append({"rect": rect, "label": label, "action": action})
        color = (78, 92, 62) if action == "go" else (62, 62, 62)
        if action == "stop":
            color = (72, 58, 58)
        self.draw_labeled_rect(image, rect, label, color)

    def draw_labeled_rect(self, image, rect, label, color):
        x, y, w, h = rect
        cv2.rectangle(image, (x, y), (x + w, y + h), color, -1)
        cv2.rectangle(image, (x, y), (x + w, y + h), (170, 170, 170), 1)
        text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.68, 2)
        self.put_text(
            image,
            label,
            (x + (w - text_size[0]) // 2, y + (h + text_size[1]) // 2),
            scale=0.68,
            thickness=2,
        )

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for index, (_, rect) in enumerate(self.field_rects):
                rx, ry, rw, rh = rect
                if rx <= x <= rx + rw and ry <= y <= ry + rh:
                    self.active_field = index
                    return
            for button in self.buttons:
                bx, by, bw, bh = button["rect"]
                if bx <= x <= bx + bw and by <= y <= by + bh:
                    action = button["action"]
                    if action == "jog":
                        self.mouse_button_command = button["label"]
                        self.set_command(button["linear"], button["angular"], f"mouse {button['label']}")
                    elif action == "copy":
                        self.copy_current_pose_to_target()
                    elif action == "go":
                        self.start_target_motion()
                    elif action == "stop":
                        self.stop()
                    return
        if event in (cv2.EVENT_LBUTTONUP, cv2.EVENT_RBUTTONDOWN):
            self.mouse_button_command = None
            if not self.target_running:
                self.stop()

    def handle_key(self, key):
        name = self.key_name(key)
        char = self.ascii_char(key)
        self.last_key_text = f"{key} ({name or char or 'unknown'})"
        if bool(self.get_parameter("log_key_codes").value):
            self.get_logger().info(f"Jog GUI key: {self.last_key_text}")

        if char == "q" or key == 27:
            return False
        if char in (" ", "s"):
            self.stop()
            return True
        if char == "\t" or key == 9:
            self.active_field = (self.active_field + 1) % len(FIELD_NAMES)
            return True
        if char == "m":
            self.rotation_mode = not self.rotation_mode
            self.last_command_text = f"mode {'rotation' if self.rotation_mode else 'translation'}"
            return True
        if char == "c":
            self.copy_current_pose_to_target()
            return True
        if char == "g":
            self.start_target_motion()
            return True
        if self.handle_text_input(key):
            return True

        linear, angular = self.command_from_key(name)
        if linear is not None:
            self.target_running = False
            self.set_command(linear, angular, f"key {name}")
        return True

    def handle_text_input(self, key):
        char = self.ascii_char(key)
        field_name = FIELD_NAMES[self.active_field]
        if key in (8, 127) or key == 0xFF08:
            self.target_fields[field_name] = self.target_fields[field_name][:-1]
            self.update_target_values_from_fields()
            return True
        if char in "0123456789.-+":
            text = self.target_fields[field_name]
            if char in ".-" and char in text:
                return True
            if char in "+-" and text:
                return True
            self.target_fields[field_name] = (text + char)[-12:]
            self.update_target_values_from_fields()
            return True
        return False

    def command_from_key(self, name):
        if name is None:
            return None, None
        zero = np.zeros(3, dtype=np.float64)
        translation = {
            "left": np.array([-1.0, 0.0, 0.0]),
            "right": np.array([1.0, 0.0, 0.0]),
            "up": np.array([0.0, 1.0, 0.0]),
            "down": np.array([0.0, -1.0, 0.0]),
            "page_up": np.array([0.0, 0.0, 1.0]),
            "page_down": np.array([0.0, 0.0, -1.0]),
        }
        rotation = {
            "left": np.array([1.0, 0.0, 0.0]),
            "right": np.array([-1.0, 0.0, 0.0]),
            "up": np.array([0.0, 1.0, 0.0]),
            "down": np.array([0.0, -1.0, 0.0]),
            "page_up": np.array([0.0, 0.0, 1.0]),
            "page_down": np.array([0.0, 0.0, -1.0]),
            "i": np.array([0.0, 1.0, 0.0]),
            "k": np.array([0.0, -1.0, 0.0]),
            "j": np.array([1.0, 0.0, 0.0]),
            "l": np.array([-1.0, 0.0, 0.0]),
            "u": np.array([0.0, 0.0, 1.0]),
            "o": np.array([0.0, 0.0, -1.0]),
        }
        if name in ("i", "j", "k", "l", "u", "o") or self.rotation_mode:
            angular = rotation.get(name)
            return (zero, angular) if angular is not None else (None, None)
        linear = translation.get(name)
        return (linear, zero) if linear is not None else (None, None)

    def set_command(self, linear_unit, angular_unit, label):
        linear_speed = float(self.get_parameter("linear_velocity").value)
        angular_speed = float(self.get_parameter("angular_velocity").value)
        self.target_linear = np.asarray(linear_unit, dtype=np.float64) * linear_speed
        self.target_angular = np.asarray(angular_unit, dtype=np.float64) * angular_speed
        self.last_command_time = time.monotonic()
        self.last_command_text = (
            f"{label}: lin=[{self.target_linear[0]:.3f}, {self.target_linear[1]:.3f}, {self.target_linear[2]:.3f}] "
            f"ang=[{self.target_angular[0]:.3f}, {self.target_angular[1]:.3f}, {self.target_angular[2]:.3f}]"
        )

    def update_output(self):
        now = time.monotonic()
        dt = max(1.0e-4, now - self.last_update_time)
        self.last_update_time = now

        if self.target_running:
            self.update_target_motion(now)
        else:
            if self.mouse_button_command is not None:
                self.last_command_time = now
            if now - self.last_command_time > float(self.get_parameter("hold_timeout").value):
                self.target_linear = np.zeros(3, dtype=np.float64)
                self.target_angular = np.zeros(3, dtype=np.float64)

        self.current_linear = slew_vector(
            self.current_linear,
            self.target_linear,
            float(self.get_parameter("linear_acceleration").value) * dt,
        )
        self.current_angular = slew_vector(
            self.current_angular,
            self.target_angular,
            float(self.get_parameter("angular_acceleration").value) * dt,
        )
        if (
            np.linalg.norm(self.current_linear) > 1.0e-5
            or np.linalg.norm(self.current_angular) > 1.0e-5
            or np.linalg.norm(self.target_linear) > 1.0e-5
            or np.linalg.norm(self.target_angular) > 1.0e-5
        ):
            self.publish_twist(self.current_linear, self.current_angular)

    def update_target_motion(self, now):
        if self.current_pose is None:
            self.last_command_text = "target stopped: TF missing"
            self.stop()
            return
        if now - self.target_started > float(self.get_parameter("target_timeout").value):
            self.last_command_text = "target timeout"
            self.stop()
            return

        current_position, current_quat = self.current_pose
        target_position = self.target_values[:3]
        target_quat = quat_from_rpy(np.radians(self.target_values[3:6]))
        linear_error = target_position - current_position
        q_error = quat_multiply(target_quat, quat_inverse(current_quat))
        rotvec_error = quat_to_rotvec(q_error)
        self.target_linear_error = float(np.linalg.norm(linear_error))
        self.target_angular_error = float(np.linalg.norm(rotvec_error))

        pos_tol = float(self.get_parameter("target_position_tolerance").value)
        rot_tol = math.radians(float(self.get_parameter("target_orientation_tolerance_deg").value))
        if self.target_linear_error <= pos_tol and self.target_angular_error <= rot_tol:
            self.last_command_text = "target reached"
            self.stop()
            return

        linear = linear_error * float(self.get_parameter("target_position_gain").value)
        angular = rotvec_error * float(self.get_parameter("target_orientation_gain").value)
        self.target_linear = limit_norm(
            linear, float(self.get_parameter("target_max_linear_velocity").value)
        )
        self.target_angular = limit_norm(
            angular, float(self.get_parameter("target_max_angular_velocity").value)
        )
        self.last_command_time = now
        self.last_command_text = (
            f"go target: pos_err={self.target_linear_error:.4f} m "
            f"rot_err={math.degrees(self.target_angular_error):.2f} deg"
        )

    def start_target_motion(self):
        self.update_target_values_from_fields()
        if self.current_pose is None:
            self.last_command_text = "target unavailable: no TF"
            return
        if self.publisher.get_subscription_count() <= 0:
            self.last_command_text = "target unavailable: no subscriber"
            return
        self.target_running = True
        self.target_started = time.monotonic()
        self.current_linear = np.zeros(3, dtype=np.float64)
        self.current_angular = np.zeros(3, dtype=np.float64)
        self.last_command_text = "go target"

    def copy_current_pose_to_target(self):
        if self.current_pose is None:
            self.last_command_text = "copy unavailable: no TF"
            return
        position, quat = self.current_pose
        rpy_deg = np.degrees(quat_to_rpy(quat))
        values = list(position) + list(rpy_deg)
        self.target_values = np.asarray(values, dtype=np.float64)
        for name, value in zip(FIELD_NAMES, self.target_values):
            self.target_fields[name] = f"{value:.4f}"
        self.last_command_text = "copied current pose"

    def update_target_values_from_fields(self):
        values = []
        for name in FIELD_NAMES:
            try:
                values.append(float(self.target_fields[name]))
            except ValueError:
                values.append(0.0)
        self.target_values = np.asarray(values, dtype=np.float64)

    def publish_twist(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.twist.linear.x = float(linear[0])
        msg.twist.linear.y = float(linear[1])
        msg.twist.linear.z = float(linear[2])
        msg.twist.angular.x = float(angular[0])
        msg.twist.angular.y = float(angular[1])
        msg.twist.angular.z = float(angular[2])
        self.publisher.publish(msg)

    def stop(self):
        self.target_running = False
        self.target_linear = np.zeros(3, dtype=np.float64)
        self.target_angular = np.zeros(3, dtype=np.float64)
        self.current_linear = np.zeros(3, dtype=np.float64)
        self.current_angular = np.zeros(3, dtype=np.float64)
        self.last_command_text = "stopped"
        for _ in range(3):
            self.publish_twist(self.current_linear, self.current_angular)
            time.sleep(0.005)

    def update_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.tf_base_frame, self.tf_tip_frame, Time()
            )
        except TransformException as exc:
            self.current_pose = None
            self.last_tf_error = str(exc)
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.current_pose = (
            np.array([translation.x, translation.y, translation.z], dtype=np.float64),
            normalize_quaternion([rotation.x, rotation.y, rotation.z, rotation.w]),
        )
        self.current_pose_stamp_monotonic = time.monotonic()
        self.last_tf_error = ""
        if not self.target_initialized:
            position, quat = self.current_pose
            rpy_deg = np.degrees(quat_to_rpy(quat))
            self.target_values = np.asarray(list(position) + list(rpy_deg), dtype=np.float64)
            for name, value in zip(FIELD_NAMES, self.target_values):
                self.target_fields[name] = f"{value:.4f}"
            self.target_initialized = True

    def joint_state_callback(self, msg):
        found = False
        for index, name in enumerate(msg.name):
            if name not in self.joint_names:
                continue
            if index < len(msg.position) and math.isfinite(msg.position[index]):
                self.joint_positions[name] = msg.position[index]
                found = True
        if found:
            self.joint_stamp_monotonic = time.monotonic()

    def equilibrium_pose_callback(self, msg):
        self.equilibrium_pose = pose_msg_to_tuple(msg)

    def target_pose_callback(self, msg):
        self.target_pose_topic_pose = pose_msg_to_tuple(msg)

    def collision_status_callback(self, msg):
        self.collision_status = msg.data

    def collision_clearance_callback(self, msg):
        self.collision_clearance = msg.data

    def key_name(self, key):
        arrow_codes = {
            81: "left",
            82: "up",
            83: "right",
            84: "down",
            85: "page_up",
            86: "page_down",
            2424832: "left",
            2490368: "up",
            2555904: "right",
            2621440: "down",
            2162688: "page_up",
            2228224: "page_down",
            65361: "left",
            65362: "up",
            65363: "right",
            65364: "down",
            65365: "page_up",
            65366: "page_down",
            0x01000012: "left",
            0x01000013: "up",
            0x01000014: "right",
            0x01000015: "down",
            0x01000016: "page_up",
            0x01000017: "page_down",
            0x250000: "left",
            0x260000: "up",
            0x270000: "right",
            0x280000: "down",
            0x210000: "page_up",
            0x220000: "page_down",
        }
        candidates = {
            key,
            key & 0xFFFF,
            key & 0xFFFFFF,
            key & 0x01FFFFFF,
            key & ~0x04000000,
            key & ~0x00100000,
            key & ~0x00040000,
        }
        for candidate in candidates:
            if candidate in arrow_codes:
                return arrow_codes[candidate]
        char = self.ascii_char(key)
        if char in ("i", "j", "k", "l", "u", "o"):
            return char
        return None

    @staticmethod
    def ascii_char(key):
        if 0 <= key < 128:
            return chr(key).lower()
        return ""

    def put_text(self, image, text, origin, scale=0.62, color=(255, 255, 255), thickness=1):
        cv2.putText(
            image,
            text,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (0, 0, 0),
            thickness + 2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            text,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    def param_or_default(self, name, default):
        value = str(self.get_parameter(name).value)
        return default if value == "" else value


def pose_msg_to_tuple(msg):
    position = msg.pose.position
    orientation = msg.pose.orientation
    return (
        np.array([position.x, position.y, position.z], dtype=np.float64),
        normalize_quaternion([orientation.x, orientation.y, orientation.z, orientation.w]),
    )


def slew_vector(current, target, max_delta):
    current = np.asarray(current, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    delta = target - current
    delta_norm = np.linalg.norm(delta)
    if delta_norm <= max_delta or delta_norm < 1.0e-12:
        return target.copy()
    return current + delta * (max_delta / delta_norm)


def limit_norm(values, limit):
    values = np.asarray(values, dtype=np.float64)
    norm = np.linalg.norm(values)
    if limit <= 0.0 or norm <= limit or norm < 1.0e-12:
        return values
    return values * (limit / norm)


def normalize_quaternion(quaternion):
    q = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1.0e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    q = q / norm
    if q[3] < 0.0:
        q = -q
    return q


def quat_inverse(q):
    q = normalize_quaternion(q)
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize_quaternion([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_to_rotvec(q):
    q = normalize_quaternion(q)
    vector_norm = np.linalg.norm(q[:3])
    if vector_norm < 1.0e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * math.atan2(vector_norm, q[3])
    if angle > math.pi:
        angle -= 2.0 * math.pi
    axis = q[:3] / vector_norm
    return axis * angle


def quat_from_rpy(rpy):
    roll, pitch, yaw = rpy
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return normalize_quaternion([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quat_to_rpy(q):
    x, y, z, w = normalize_quaternion(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw], dtype=np.float64)


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedCartesianJogGui()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
