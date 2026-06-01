#!/usr/bin/env python3

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ur_dashboard_msgs.action import SetMode


def parse_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class URStartupEnable(Node):
    def __init__(self):
        super().__init__('ur_startup_enable')
        self.declare_parameter('arm_namespace', '')
        self.declare_parameter('wait_timeout', 60.0)
        self.declare_parameter('target_robot_mode', 7)
        self.declare_parameter('stop_program', True)
        self.declare_parameter('play_program', True)

        arm_namespace = self.get_parameter('arm_namespace').value.strip('/')
        if not arm_namespace:
            raise RuntimeError('arm_namespace must not be empty')

        action_name = f'/{arm_namespace}/ur_robot_state_helper/set_mode'
        self._wait_timeout = float(self.get_parameter('wait_timeout').value)
        self._target_robot_mode = int(self.get_parameter('target_robot_mode').value)
        self._stop_program = parse_bool(self.get_parameter('stop_program').value)
        self._play_program = parse_bool(self.get_parameter('play_program').value)
        self._client = ActionClient(self, SetMode, action_name)
        self.get_logger().info(f'Waiting for UR SetMode action: {action_name}')

    def run(self):
        if not self._client.wait_for_server(timeout_sec=self._wait_timeout):
            self.get_logger().error('UR SetMode action did not become available.')
            return 1

        goal = SetMode.Goal()
        goal.target_robot_mode = self._target_robot_mode
        goal.stop_program = self._stop_program
        goal.play_program = self._play_program

        self.get_logger().info(
            f'Sending UR SetMode goal: target_robot_mode={goal.target_robot_mode}, '
            f'stop_program={goal.stop_program}, play_program={goal.play_program}'
        )
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('UR SetMode goal was rejected.')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'UR SetMode succeeded: {result.message}')
            return 0

        self.get_logger().error(f'UR SetMode failed: {result.message}')
        return 1


def main():
    rclpy.init()
    node = None
    try:
        node = URStartupEnable()
        return node.run()
    except Exception as exc:  # noqa: BLE001
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(str(exc), file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
