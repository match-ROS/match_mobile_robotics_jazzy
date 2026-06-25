#!/usr/bin/env python3
import http.client
import json
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

DEFAULT_AUTH = (
    'Basic '
    'ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='
)


def _normalize_host(hostname):
    parsed = urlparse(hostname if '://' in hostname else 'http://' + hostname)
    return parsed.hostname or hostname


class MiRBatteryStatePublisher(Node):
    def __init__(self):
        super().__init__('mir_battery_state_publisher')
        self.robot_ip = self.declare_parameter('mir_hostname', '192.168.12.20').value
        self.auth = self.declare_parameter('mir_restapi_auth', DEFAULT_AUTH).value
        self.period = float(self.declare_parameter('period', 2.0).value)
        self.timeout = float(self.declare_parameter('timeout', 3.0).value)
        self.failed_reads = 0
        self.publisher = self.create_publisher(BatteryState, 'battery_state', 1)
        self.timer = self.create_timer(self.period, self.query_status)

    def query_status(self):
        headers = {
            'Authorization': self.auth,
            'Accept': 'application/json',
            'Accept-Language': 'en_US',
        }
        host = _normalize_host(str(self.robot_ip))
        connection = http.client.HTTPConnection(host=host, port=80, timeout=self.timeout)
        try:
            connection.request('GET', '/api/v2.0.0/status', headers=headers)
            response = connection.getresponse()
            if response.status < 200 or response.status >= 300:
                raise RuntimeError('HTTP {} {}'.format(response.status, response.reason))
            data = json.loads(response.read().decode('utf-8'))
            percentage = float(data.get('battery_percentage', -1.0))

            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.percentage = round(percentage / 100.0, 4)
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            self.publisher.publish(msg)
            self.failed_reads = 0
            self.get_logger().info('[{}] Battery: {:.2f}%'.format(host, percentage), throttle_duration_sec=10.0)
        except Exception as exc:
            self.failed_reads += 1
            message = 'Battery read failed from {}: {}'.format(host, exc)
            if self.failed_reads < 2:
                self.get_logger().debug(message)
            else:
                self.get_logger().warning(message, throttle_duration_sec=10.0)
        finally:
            connection.close()


def main(args=None):
    rclpy.init(args=args)
    node = MiRBatteryStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
