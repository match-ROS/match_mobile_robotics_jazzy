#!/usr/bin/env python3
"""Guarded DS4 teleoperation for the MiR base."""

import math

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class Ds4MirTeleop(Node):
    def __init__(self):
        super().__init__('standalone_mir_teleop')

        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('l1_button', 4)
        self.declare_parameter('r1_button', 5)
        self.declare_parameter('max_linear_mps', 0.25)
        self.declare_parameter('max_angular_radps', 0.45)
        self.declare_parameter('initial_linear_mps', 0.10)
        self.declare_parameter('initial_angular_radps', 0.20)
        self.declare_parameter('min_linear_mps', 0.02)
        self.declare_parameter('min_angular_radps', 0.05)
        self.declare_parameter('limit_step_fraction', 0.05)
        self.declare_parameter('linear_increase_button', 3)
        self.declare_parameter('linear_decrease_button', 1)
        self.declare_parameter('angular_decrease_button', 0)
        self.declare_parameter('angular_increase_button', 2)
        self.declare_parameter('deadzone', 0.10)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('joy_timeout_sec', 0.30)
        self.declare_parameter('frame_id', 'mur620d/base_link')
        self.declare_parameter('joy_topic', '/mur620d/joy')
        self.declare_parameter('cmd_vel_topic', '/mur620d/cmd_vel_stamped')
        self.declare_parameter('status_topic', '/mur620d/standalone_mir_teleop/status')

        self.linear_axis = int(self.get_parameter('linear_axis').value)
        self.angular_axis = int(self.get_parameter('angular_axis').value)
        self.l1_button = int(self.get_parameter('l1_button').value)
        self.r1_button = int(self.get_parameter('r1_button').value)
        self.max_linear_mps = float(self.get_parameter('max_linear_mps').value)
        self.max_angular_radps = float(self.get_parameter('max_angular_radps').value)
        self.min_linear_mps = max(0.0, float(self.get_parameter('min_linear_mps').value))
        self.min_angular_radps = max(0.0, float(self.get_parameter('min_angular_radps').value))
        self.limit_step_factor = 1.0 + max(0.0, float(self.get_parameter('limit_step_fraction').value))
        self.linear_increase_button = int(self.get_parameter('linear_increase_button').value)
        self.linear_decrease_button = int(self.get_parameter('linear_decrease_button').value)
        self.angular_decrease_button = int(self.get_parameter('angular_decrease_button').value)
        self.angular_increase_button = int(self.get_parameter('angular_increase_button').value)
        self.current_linear_mps = self.clamp(
            float(self.get_parameter('initial_linear_mps').value),
            self.min_linear_mps,
            self.max_linear_mps,
        )
        self.current_angular_radps = self.clamp(
            float(self.get_parameter('initial_angular_radps').value),
            self.min_angular_radps,
            self.max_angular_radps,
        )
        self.deadzone = max(0.0, min(0.95, float(self.get_parameter('deadzone').value)))
        self.joy_timeout_sec = max(0.05, float(self.get_parameter('joy_timeout_sec').value))
        self.frame_id = str(self.get_parameter('frame_id').value)
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        joy_topic = str(self.get_parameter('joy_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)

        self.last_joy = None
        self.last_joy_time = None
        self.last_status = None
        self.previous_buttons = []

        self.cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_command)

        self.get_logger().info(
            f'Guarded MiR teleop: {joy_topic} -> {cmd_vel_topic}; '
            f'hold buttons {self.l1_button}+{self.r1_button} to drive.'
        )
        self.log_limits()
        self.publish_status('waiting_for_joy')

    def joy_callback(self, msg):
        self.apply_limit_buttons(msg)
        self.last_joy = msg
        self.last_joy_time = self.get_clock().now()
        self.previous_buttons = list(msg.buttons)

    def publish_command(self):
        linear_x = 0.0
        angular_z = 0.0
        status = self.current_status()

        if status == 'armed':
            linear_x = self.scaled_axis(self.linear_axis, self.current_linear_mps)
            angular_z = self.scaled_axis(self.angular_axis, self.current_angular_radps)

        self.publish_twist(linear_x, angular_z)
        self.publish_status(status)

    def current_status(self):
        if self.last_joy is None or self.last_joy_time is None:
            return 'waiting_for_joy'

        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        if age > self.joy_timeout_sec:
            return 'joy_timeout'

        if self.button_pressed(self.l1_button) and self.button_pressed(self.r1_button):
            return 'armed'
        return 'disarmed'

    def button_pressed(self, index):
        return 0 <= index < len(self.last_joy.buttons) and bool(self.last_joy.buttons[index])

    def button_rising(self, buttons, index):
        if index < 0 or index >= len(buttons):
            return False
        previous = index < len(self.previous_buttons) and bool(self.previous_buttons[index])
        return bool(buttons[index]) and not previous

    def apply_limit_buttons(self, joy_msg):
        changed = False
        buttons = joy_msg.buttons
        if self.button_rising(buttons, self.linear_increase_button):
            self.current_linear_mps = self.clamp(
                self.current_linear_mps * self.limit_step_factor,
                self.min_linear_mps,
                self.max_linear_mps,
            )
            changed = True
        if self.button_rising(buttons, self.linear_decrease_button):
            self.current_linear_mps = self.clamp(
                self.current_linear_mps / self.limit_step_factor,
                self.min_linear_mps,
                self.max_linear_mps,
            )
            changed = True
        if self.button_rising(buttons, self.angular_increase_button):
            self.current_angular_radps = self.clamp(
                self.current_angular_radps * self.limit_step_factor,
                self.min_angular_radps,
                self.max_angular_radps,
            )
            changed = True
        if self.button_rising(buttons, self.angular_decrease_button):
            self.current_angular_radps = self.clamp(
                self.current_angular_radps / self.limit_step_factor,
                self.min_angular_radps,
                self.max_angular_radps,
            )
            changed = True
        if changed:
            self.log_limits()

    def clamp(self, value, minimum, maximum):
        if maximum < minimum:
            return minimum
        return min(max(value, minimum), maximum)

    def log_limits(self):
        self.get_logger().info(
            f'MiR teleop limits: linear={self.current_linear_mps:.3f} m/s '
            f'angular={self.current_angular_radps:.3f} rad/s'
        )

    def scaled_axis(self, index, scale):
        if self.last_joy is None or index < 0 or index >= len(self.last_joy.axes):
            return 0.0
        value = float(self.last_joy.axes[index])
        if not math.isfinite(value) or abs(value) < self.deadzone:
            return 0.0
        normalized = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return math.copysign(normalized, value) * scale

    def publish_twist(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def publish_status(self, status):
        if status == self.last_status:
            return
        self.last_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'MiR teleop status: {status}')

    def send_stop(self, count=3):
        for _ in range(count):
            try:
                self.publish_twist(0.0, 0.0)
            except RCLError:
                break


def main():
    rclpy.init()
    node = Ds4MirTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.send_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
