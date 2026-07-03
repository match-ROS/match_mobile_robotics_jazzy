#!/usr/bin/env python3
"""Guarded DS4 teleoperation for the MiR base."""

import math

import rclpy
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

        self.cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_command)

        self.get_logger().info(
            f'Guarded MiR teleop: {joy_topic} -> {cmd_vel_topic}; '
            f'hold buttons {self.l1_button}+{self.r1_button} to drive.'
        )
        self.publish_status('waiting_for_joy')

    def joy_callback(self, msg):
        self.last_joy = msg
        self.last_joy_time = self.get_clock().now()

    def publish_command(self):
        linear_x = 0.0
        angular_z = 0.0
        status = self.current_status()

        if status == 'armed':
            linear_x = self.scaled_axis(self.linear_axis, self.max_linear_mps)
            angular_z = self.scaled_axis(self.angular_axis, self.max_angular_radps)

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
            self.publish_twist(0.0, 0.0)


def main():
    rclpy.init()
    node = Ds4MirTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.send_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
