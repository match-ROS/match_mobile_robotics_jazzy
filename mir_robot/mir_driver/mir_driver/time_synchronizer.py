#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class TimeSynchronizer(Node):
    def __init__(self):
        super().__init__('mir_time_synchronizer')
        self.service_name = self.declare_parameter('service_name', 'mir_sync_time').value
        self.timeout = float(self.declare_parameter('timeout', 60.0).value)
        self.client = self.create_client(Trigger, self.service_name)

    def run(self):
        if not self.client.wait_for_service(timeout_sec=self.timeout):
            self.get_logger().error('Service {} is not available'.format(self.service_name))
            return 1

        future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        if not future.done():
            self.get_logger().error('Timed out waiting for {}'.format(self.service_name))
            return 1

        result = future.result()
        if result is None:
            self.get_logger().error('No response from {}'.format(self.service_name))
            return 1
        if result.success:
            self.get_logger().info(result.message)
            return 0

        self.get_logger().error(result.message)
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = TimeSynchronizer()
    exit_code = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
