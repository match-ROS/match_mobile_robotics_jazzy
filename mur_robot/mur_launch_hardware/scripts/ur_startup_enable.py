#!/usr/bin/env python3

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from ur_dashboard_msgs.action import SetMode
from ur_dashboard_msgs.srv import IsProgramRunning


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
        self.declare_parameter('verify_program_running', True)

        arm_namespace = self.get_parameter('arm_namespace').value.strip('/')
        if not arm_namespace:
            raise RuntimeError('arm_namespace must not be empty')

        action_name = f'/{arm_namespace}/ur_robot_state_helper/set_mode'
        dashboard_play_name = f'/{arm_namespace}/dashboard_client/play'
        program_running_name = f'/{arm_namespace}/dashboard_client/program_running'
        self._wait_timeout = float(self.get_parameter('wait_timeout').value)
        self._target_robot_mode = int(self.get_parameter('target_robot_mode').value)
        self._stop_program = parse_bool(self.get_parameter('stop_program').value)
        self._play_program = parse_bool(self.get_parameter('play_program').value)
        self._verify_program_running = parse_bool(
            self.get_parameter('verify_program_running').value
        )
        self._client = ActionClient(self, SetMode, action_name)
        self._dashboard_play_client = self.create_client(Trigger, dashboard_play_name)
        self._program_running_client = self.create_client(
            IsProgramRunning, program_running_name
        )
        self.get_logger().info(f'Waiting for UR SetMode action: {action_name}')
        self.get_logger().info(f'Using UR Dashboard play service: {dashboard_play_name}')

    def _call_program_running(self):
        future = self._program_running_client.call_async(IsProgramRunning.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            return None
        return future.result()

    def _wait_for_program_running(self, timeout):
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1.0e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            response = self._call_program_running()
            if response is not None and response.success and response.program_running:
                self.get_logger().info('UR Dashboard reports program_running=true.')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _ensure_program_running(self):
        if not self._play_program or not self._verify_program_running:
            return True

        service_timeout = min(self._wait_timeout, 10.0)
        if not self._program_running_client.wait_for_service(timeout_sec=service_timeout):
            self.get_logger().error('UR Dashboard program_running service did not become available.')
            return False

        response = self._call_program_running()
        if response is not None and response.success and response.program_running:
            self.get_logger().info('UR program is already running.')
            return True

        if response is not None:
            self.get_logger().warn(f'UR program is not running yet: {response.answer}')

        if not self._dashboard_play_client.wait_for_service(timeout_sec=service_timeout):
            self.get_logger().error('UR Dashboard play service did not become available.')
            return False

        self.get_logger().info('Calling UR Dashboard play service.')
        play_future = self._dashboard_play_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, play_future, timeout_sec=service_timeout)
        play_response = play_future.result()
        if play_response is None or not play_response.success:
            message = play_response.message if play_response is not None else 'no response'
            self.get_logger().error(f'UR Dashboard play failed: {message}')
            return False

        self.get_logger().info(f'UR Dashboard play accepted: {play_response.message}')
        if self._wait_for_program_running(service_timeout):
            return True

        self.get_logger().error('UR program did not report running after Dashboard play.')
        return False

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
            return 0 if self._ensure_program_running() else 1
        if 'Reached target robot mode' in result.message:
            self.get_logger().warn(
                f'UR SetMode reached the requested robot mode, but reported: {result.message}'
            )
            return 0 if self._ensure_program_running() else 1

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
