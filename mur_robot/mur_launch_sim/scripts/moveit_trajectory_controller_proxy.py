#!/usr/bin/env python3
"""Proxy FollowJointTrajectory goals through an automatic controller switch."""

import argparse
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


JOINT_COUNT = 6


class MoveItTrajectoryControllerProxy(Node):
    def __init__(self, args):
        super().__init__(f'moveit_trajectory_controller_proxy_{args.arm}')
        self.args = args
        self.callback_group = ReentrantCallbackGroup()
        self._active_client_goal = None
        self._client_goal_lock = threading.Lock()

        self.proxy_action = (
            f'/{args.robot_name}/moveit_joint_trajectory_controller_{args.arm}/'
            'follow_joint_trajectory'
        )
        self.real_action = (
            f'/{args.robot_name}/joint_trajectory_controller_{args.arm}/'
            'follow_joint_trajectory'
        )
        self.controller_manager = f'/{args.robot_name}/controller_manager'
        self.velocity_controller = f'forward_velocity_controller_{args.arm}'
        self.trajectory_controller = f'joint_trajectory_controller_{args.arm}'
        self.velocity_command_topic = (
            f'/{args.robot_name}/{self.velocity_controller}/commands'
        )

        self.switch_client = self.create_client(
            SwitchController,
            f'{self.controller_manager}/switch_controller',
            callback_group=self.callback_group,
        )
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.real_action,
            callback_group=self.callback_group,
        )
        self.zero_velocity_pub = self.create_publisher(
            Float64MultiArray,
            self.velocity_command_topic,
            10,
        )

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.proxy_action,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f'Proxy ready: {self.proxy_action} -> {self.real_action}'
        )

    def goal_callback(self, _goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        with self._client_goal_lock:
            client_goal = self._active_client_goal
        if client_goal is not None:
            client_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _wait_for_future(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return future.done()

    def _duration_msg(self, seconds):
        duration = SwitchController.Request().timeout
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - int(seconds)) * 1_000_000_000)
        return duration

    def _switch_controllers(self, activate, deactivate, reason):
        if not self.switch_client.wait_for_service(timeout_sec=self.args.switch_timeout):
            self.get_logger().error(
                f'Controller manager service not available: '
                f'{self.controller_manager}/switch_controller'
            )
            return False

        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = self._duration_msg(self.args.switch_timeout)

        self.get_logger().info(
            f'{reason}: activate={activate}, deactivate={deactivate}'
        )
        future = self.switch_client.call_async(request)
        if not self._wait_for_future(future, self.args.switch_timeout + 1.0):
            self.get_logger().error(f'Timeout while switching controllers for {reason}')
            return False

        response = future.result()
        if response is None or not response.ok:
            message = '' if response is None else response.message
            self.get_logger().error(f'Controller switch failed for {reason}: {message}')
            return False

        return True

    def _publish_zero_velocity(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * JOINT_COUNT
        for _ in range(3):
            self.zero_velocity_pub.publish(msg)
            time.sleep(0.02)

    def _switch_to_trajectory(self):
        self._publish_zero_velocity()
        return self._switch_controllers(
            activate=[self.trajectory_controller],
            deactivate=[self.velocity_controller],
            reason='MoveIt execution start',
        )

    def _switch_to_velocity(self):
        ok = self._switch_controllers(
            activate=[self.velocity_controller],
            deactivate=[self.trajectory_controller],
            reason='MoveIt execution finished',
        )
        if ok:
            self._publish_zero_velocity()
        return ok

    def execute_callback(self, goal_handle):
        result = FollowJointTrajectory.Result()

        if not self._switch_to_trajectory():
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Could not activate trajectory controller'
            goal_handle.abort()
            return result

        try:
            if not self.trajectory_client.wait_for_server(
                timeout_sec=self.args.action_timeout
            ):
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = f'Real action server not available: {self.real_action}'
                goal_handle.abort()
                return result

            send_future = self.trajectory_client.send_goal_async(
                goal_handle.request,
                feedback_callback=lambda feedback: goal_handle.publish_feedback(
                    feedback.feedback
                ),
            )
            if not self._wait_for_future(send_future, self.args.action_timeout):
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = 'Timed out sending goal to real controller'
                goal_handle.abort()
                return result

            client_goal = send_future.result()
            if client_goal is None or not client_goal.accepted:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = 'Real controller rejected trajectory goal'
                goal_handle.abort()
                return result

            with self._client_goal_lock:
                self._active_client_goal = client_goal

            result_future = client_goal.get_result_async()
            while rclpy.ok() and not result_future.done():
                if goal_handle.is_cancel_requested:
                    client_goal.cancel_goal_async()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = 'Trajectory execution canceled'
                    return result
                time.sleep(0.02)

            real_result = result_future.result()
            if real_result is None:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = 'No result from real controller'
                goal_handle.abort()
                return result

            result = real_result.result
            if real_result.status == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
            elif real_result.status == GoalStatus.STATUS_CANCELED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        finally:
            with self._client_goal_lock:
                self._active_client_goal = None
            self._switch_to_velocity()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620a')
    parser.add_argument('--arm', choices=['l', 'r'], required=True)
    parser.add_argument('--switch-timeout', type=float, default=5.0)
    parser.add_argument('--action-timeout', type=float, default=10.0)
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = MoveItTrajectoryControllerProxy(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
