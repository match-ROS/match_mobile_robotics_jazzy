#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

import mir_driver.rosbridge


class MiRRosapiAudit(Node):
    def __init__(self):
        super().__init__('mir_rosapi_audit')
        self.hostname = self.declare_parameter('mir_hostname', '192.168.12.20').value
        self.port = int(self.declare_parameter('mir_port', 9090).value)
        self.connect_timeout = float(self.declare_parameter('connect_timeout', 5.0).value)
        self.service_timeout = float(self.declare_parameter('service_timeout', 5.0).value)

    def run(self):
        robot = mir_driver.rosbridge.RosbridgeSetup(str(self.hostname), self.port)
        deadline = time.monotonic() + self.connect_timeout
        while time.monotonic() < deadline:
            if robot.is_connected():
                break
            if robot.is_errored():
                self.get_logger().error('Connection error to {}:{}'.format(self.hostname, self.port))
                return 1
            time.sleep(0.05)
        else:
            self.get_logger().error('Timed out connecting to {}:{}'.format(self.hostname, self.port))
            return 1

        self._log_topics(robot)
        self._log_services(robot)
        return 0

    def _log_topics(self, robot):
        topics = robot.callService('/rosapi/topics', msg={}, timeout=self.service_timeout).get('topics', [])
        self.get_logger().info('MiR publishes {} ROS1 topics visible via rosapi'.format(len(topics)))
        for topic in sorted(topics):
            try:
                topic_type = robot.callService('/rosapi/topic_type', msg={'topic': topic}, timeout=self.service_timeout).get('type', '')
            except Exception as exc:
                topic_type = 'type lookup failed: {}'.format(exc)
            self.get_logger().info('topic: {} [{}]'.format(topic, topic_type))

    def _log_services(self, robot):
        try:
            services = robot.callService('/rosapi/services', msg={}, timeout=self.service_timeout).get('services', [])
        except Exception as exc:
            self.get_logger().warning('Could not query ROS1 services via rosapi: {}'.format(exc))
            return
        self.get_logger().info('MiR exposes {} ROS1 services visible via rosapi'.format(len(services)))
        for service in sorted(services):
            self.get_logger().info('service: {}'.format(service))


def main(args=None):
    rclpy.init(args=args)
    node = MiRRosapiAudit()
    exit_code = node.run()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
