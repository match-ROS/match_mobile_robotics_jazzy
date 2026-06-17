#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

import mir_driver.rosbridge
from mir_msgs.msg import LightCmd
from mir_srvs.srv import ColorRGB, LightCommand


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class RGBControl(Node):
    def __init__(self):
        super().__init__('rgb_control')
        self.hostname = self.declare_parameter('mir_hostname', '192.168.12.20').value
        self.port = int(self.declare_parameter('mir_port', 9090).value)
        self.bridge_light_commands = _as_bool(self.declare_parameter('bridge_light_commands', True).value)
        self.connect_timeout = float(self.declare_parameter('connect_timeout', 3.0).value)
        self.service_timeout = float(self.declare_parameter('service_timeout', 3.0).value)
        self.set_default_on_start = _as_bool(self.declare_parameter('set_default_on_start', False).value)
        self.robot = None

        self.light_cmd_pub = self.create_publisher(LightCmd, 'new_light_cmd', 1)
        self.create_service(Trigger, 'RGB_control/rainbow_start', self.rainbow_start_callback)
        self.create_service(Trigger, 'RGB_control/rainbow_stop', self.rainbow_stop_callback)
        self.create_service(Trigger, 'RGB_control/match_color', self.match_color_callback)
        self.create_service(ColorRGB, 'RGB_control/solid_color', self.solid_color_callback)
        self.create_service(LightCommand, 'RGB_control/light_command', self.light_command_callback)

        if self.set_default_on_start:
            self._send_command('solid', '509600', 'ffffff', 'all', '', 0.0, 1000)

    def rainbow_start_callback(self, request, response):
        success, message = self._send_command('rainbow', 'ffffff', 'ffffff', 'all', '', 0.0, 1000)
        response.success = success
        response.message = message
        return response

    def rainbow_stop_callback(self, request, response):
        success, message = self._send_command('Wave', 'ffff00', 'ff0000', 'all', '', 0.0, 1000)
        response.success = success
        response.message = message
        return response

    def match_color_callback(self, request, response):
        success, message = self._send_command('solid', 'ffff00', '509600', 'all', '', 0.0, 1000)
        response.success = success
        response.message = message
        return response

    def solid_color_callback(self, request, response):
        color = '{:02x}{:02x}{:02x}'.format(request.red, request.green, request.blue)
        success, _ = self._send_command('solid', color, 'ffffff', 'all', '', 0.0, 1000)
        response.success = success
        return response

    def light_command_callback(self, request, response):
        success, message = self._send_command(
            request.command,
            request.color1,
            request.color2,
            request.leds,
            request.token,
            request.timeout,
            request.priority,
        )
        response.success = success
        response.token = message if success else ''
        return response

    def _send_command(self, effect, color1, color2, leds, token, timeout, priority):
        msg = LightCmd()
        msg.effect = effect
        msg.color1 = color1
        msg.color2 = color2
        msg.leds = leds
        msg.token = token
        msg.timeout = float(timeout)
        msg.priority = int(priority)
        self.light_cmd_pub.publish(msg)

        if not self.bridge_light_commands:
            return True, 'published locally'

        return self._call_light_srv(msg)

    def _ensure_robot(self):
        if self.robot is not None and self.robot.is_connected():
            return True
        if self.robot is not None and self.robot.is_errored():
            self.robot = None

        if self.robot is None:
            self.get_logger().info('Connecting to MiR rosbridge at {}:{} for light service'.format(self.hostname, self.port))
            self.robot = mir_driver.rosbridge.RosbridgeSetup(str(self.hostname), self.port)

        deadline = time.monotonic() + self.connect_timeout
        while time.monotonic() < deadline:
            if self.robot.is_connected():
                return True
            if self.robot.is_errored():
                return False
            time.sleep(0.05)
        return False

    def _call_light_srv(self, msg):
        if not self._ensure_robot():
            message = 'MiR rosbridge is not connected'
            self.get_logger().warning(message)
            return False, message

        args = {
            'command': msg.effect,
            'color1': msg.color1,
            'color2': msg.color2,
            'leds': msg.leds,
            'token': msg.token,
            'timeout': msg.timeout,
            'priority': msg.priority,
        }
        try:
            response = self.robot.callService('/light_srv', msg=args, timeout=self.service_timeout)
        except Exception as exc:
            message = 'Light service call failed: {}'.format(exc)
            self.get_logger().warning(message)
            return False, message

        success = bool(response.get('success', False))
        response_token = str(response.get('token', ''))
        if not success:
            self.get_logger().warning('Light service returned failure: {}'.format(response))
        return success, response_token or 'OK'


def main(args=None):
    rclpy.init(args=args)
    node = RGBControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
