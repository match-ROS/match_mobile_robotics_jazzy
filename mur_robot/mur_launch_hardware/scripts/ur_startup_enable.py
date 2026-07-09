#!/usr/bin/env python3

import sys
import time

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
        self.declare_parameter('program_running_wait_timeout', 8.0)
        self.declare_parameter('dashboard_play_retries', 1)
        self.declare_parameter('dashboard_play_retry_delay', 2.0)
        self.declare_parameter('set_mode_retries', 1)
        self.declare_parameter('recover_dashboard_blockers', True)

        arm_namespace = self.get_parameter('arm_namespace').value.strip('/')
        if not arm_namespace:
            raise RuntimeError('arm_namespace must not be empty')

        action_name = f'/{arm_namespace}/ur_robot_state_helper/set_mode'
        dashboard_play_name = f'/{arm_namespace}/dashboard_client/play'
        program_running_name = f'/{arm_namespace}/dashboard_client/program_running'
        close_safety_popup_name = f'/{arm_namespace}/dashboard_client/close_safety_popup'
        close_popup_name = f'/{arm_namespace}/dashboard_client/close_popup'
        unlock_protective_stop_name = f'/{arm_namespace}/dashboard_client/unlock_protective_stop'
        self._wait_timeout = float(self.get_parameter('wait_timeout').value)
        self._service_timeout = max(1.0, min(self._wait_timeout, 10.0))
        self._target_robot_mode = int(self.get_parameter('target_robot_mode').value)
        self._stop_program = parse_bool(self.get_parameter('stop_program').value)
        self._play_program = parse_bool(self.get_parameter('play_program').value)
        self._verify_program_running = parse_bool(
            self.get_parameter('verify_program_running').value
        )
        self._program_running_wait_timeout = max(
            0.0,
            float(self.get_parameter('program_running_wait_timeout').value),
        )
        self._dashboard_play_retries = max(
            0,
            int(self.get_parameter('dashboard_play_retries').value),
        )
        self._dashboard_play_retry_delay = max(
            0.1,
            float(self.get_parameter('dashboard_play_retry_delay').value),
        )
        self._set_mode_retries = max(0, int(self.get_parameter('set_mode_retries').value))
        self._recover_dashboard_blockers = parse_bool(
            self.get_parameter('recover_dashboard_blockers').value
        )
        self._client = ActionClient(self, SetMode, action_name)
        self._dashboard_play_client = self.create_client(Trigger, dashboard_play_name)
        self._program_running_client = self.create_client(
            IsProgramRunning, program_running_name
        )
        self._close_safety_popup_client = self.create_client(Trigger, close_safety_popup_name)
        self._close_popup_client = self.create_client(Trigger, close_popup_name)
        self._unlock_protective_stop_client = self.create_client(
            Trigger, unlock_protective_stop_name
        )
        self.get_logger().info(f'Waiting for UR SetMode action: {action_name}')
        self.get_logger().info(f'Using UR Dashboard play service: {dashboard_play_name}')

    def _wait_for_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        return future.done()

    def _sleep(self, duration):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, deadline - time.monotonic()))

    def _call_program_running(self):
        future = self._program_running_client.call_async(IsProgramRunning.Request())
        if not self._wait_for_future(future, min(self._service_timeout, 2.0)):
            self.get_logger().warn('UR Dashboard program_running query timed out.')
            return None
        if future.result() is None:
            return None
        return future.result()

    def _wait_for_program_running(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            response = self._call_program_running()
            if response is not None and response.success and response.program_running:
                self.get_logger().info('UR Dashboard reports program_running=true.')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _call_trigger(self, client, label, timeout=None, warn_only=False):
        timeout = self._service_timeout if timeout is None else timeout
        if not client.wait_for_service(timeout_sec=timeout):
            message = f'{label} service did not become available.'
            if warn_only:
                self.get_logger().warn(message)
            else:
                self.get_logger().error(message)
            return None

        future = client.call_async(Trigger.Request())
        if not self._wait_for_future(future, timeout):
            message = f'{label} service timed out.'
            if warn_only:
                self.get_logger().warn(message)
            else:
                self.get_logger().error(message)
            return None

        response = future.result()
        if response is None:
            message = f'{label} returned no response.'
            if warn_only:
                self.get_logger().warn(message)
            else:
                self.get_logger().error(message)
            return None

        if response.success:
            self.get_logger().info(f'{label} accepted: {response.message}')
        elif warn_only:
            self.get_logger().warn(f'{label} failed: {response.message}')
        else:
            self.get_logger().error(f'{label} failed: {response.message}')
        return response

    def _recover_dashboard_blockers(self):
        if not self._recover_dashboard_blockers:
            return
        self.get_logger().warn(
            'Trying to clear transient UR dashboard blockers before retrying startup.'
        )
        cleanup_steps = (
            ('UR Dashboard close safety popup', self._close_safety_popup_client),
            ('UR Dashboard close popup', self._close_popup_client),
            ('UR Dashboard unlock protective stop', self._unlock_protective_stop_client),
            ('UR Dashboard close popup', self._close_popup_client),
        )
        for label, client in cleanup_steps:
            self._call_trigger(client, label, timeout=2.0, warn_only=True)
            self._sleep(0.1)

    def _ensure_program_running(self):
        if not self._play_program or not self._verify_program_running:
            return True

        if not self._program_running_client.wait_for_service(timeout_sec=self._service_timeout):
            self.get_logger().error('UR Dashboard program_running service did not become available.')
            return False

        response = self._call_program_running()
        if response is not None and response.success and response.program_running:
            self.get_logger().info('UR program is already running.')
            return True

        if response is not None:
            self.get_logger().warn(f'UR program is not running yet: {response.answer}')

        if self._program_running_wait_timeout > 0.0:
            self.get_logger().info(
                'Waiting for SetMode play_program to report program_running=true.'
            )
            if self._wait_for_program_running(self._program_running_wait_timeout):
                return True

        attempts = self._dashboard_play_retries + 1
        for attempt in range(1, attempts + 1):
            self.get_logger().info(
                f'Calling UR Dashboard play service (attempt {attempt}/{attempts}).'
            )
            play_response = self._call_trigger(
                self._dashboard_play_client,
                'UR Dashboard play',
            )
            if play_response is not None and play_response.success:
                if self._wait_for_program_running(self._service_timeout):
                    return True
                self.get_logger().warn(
                    'UR program did not report running after Dashboard play.'
                )
            elif attempt < attempts:
                self._recover_dashboard_blockers()

            if attempt < attempts:
                self.get_logger().info(
                    f'Waiting {self._dashboard_play_retry_delay:.1f}s before retrying Dashboard play.'
                )
                if self._wait_for_program_running(self._dashboard_play_retry_delay):
                    return True

        self.get_logger().error('UR program did not report running after Dashboard play retries.')
        return False

    def _send_set_mode(self, stop_program, play_program):
        if not self._client.wait_for_server(timeout_sec=self._wait_timeout):
            self.get_logger().error('UR SetMode action did not become available.')
            return False

        goal = SetMode.Goal()
        goal.target_robot_mode = self._target_robot_mode
        goal.stop_program = stop_program
        goal.play_program = play_program

        self.get_logger().info(
            f'Sending UR SetMode goal: target_robot_mode={goal.target_robot_mode}, '
            f'stop_program={goal.stop_program}, play_program={goal.play_program}'
        )
        future = self._client.send_goal_async(goal)
        if not self._wait_for_future(future, self._wait_timeout):
            self.get_logger().error('UR SetMode goal request timed out.')
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('UR SetMode goal was rejected.')
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, self._wait_timeout):
            self.get_logger().error('UR SetMode result timed out.')
            return False
        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'UR SetMode succeeded: {result.message}')
            return True
        if 'Reached target robot mode' in result.message:
            self.get_logger().warn(
                f'UR SetMode reached the requested robot mode, but reported: {result.message}'
            )
            return True

        self.get_logger().error(f'UR SetMode failed: {result.message}')
        return False

    def run(self):
        attempts = self._set_mode_retries + 1
        for attempt in range(1, attempts + 1):
            if attempt > 1:
                self.get_logger().warn(
                    f'Retrying UR SetMode startup (attempt {attempt}/{attempts}).'
                )
                self._recover_dashboard_blockers()

            stop_program = self._stop_program if attempt == 1 else False
            if not self._send_set_mode(stop_program, self._play_program):
                if attempt < attempts:
                    continue
                return 1

            if self._ensure_program_running():
                return 0

        self.get_logger().error('UR startup did not reach a running UR program.')
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
