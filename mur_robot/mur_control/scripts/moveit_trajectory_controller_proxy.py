#!/usr/bin/env python3
"""Proxy FollowJointTrajectory goals through an automatic controller switch."""

import argparse
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


ARM_JOINT_COUNT = 6


class MoveItTrajectoryControllerProxy(Node):
    def __init__(self, args):
        super().__init__(f'moveit_trajectory_controller_proxy_{args.arm}')
        self.args = args
        self.callback_group = ReentrantCallbackGroup()
        self._active_client_goal = None
        self._client_goal_lock = threading.Lock()
        self._joint_positions = {}
        self._joint_lock = threading.Lock()

        self.proxy_action = args.proxy_action or (
            f'/{args.robot_name}/moveit_joint_trajectory_controller_{args.arm}/'
            'follow_joint_trajectory'
        )
        self.real_action = args.real_action or (
            f'/{args.robot_name}/joint_trajectory_controller_{args.arm}/'
            'follow_joint_trajectory'
        )
        self.controller_manager = args.controller_manager or f'/{args.robot_name}/controller_manager'
        self.velocity_controller = args.velocity_controller
        self.trajectory_controller = args.trajectory_controller
        self.velocity_command_topic = args.velocity_command_topic or (
            f'/{args.robot_name}/{self.velocity_controller}/commands'
        )

        self.switch_client = self.create_client(
            SwitchController,
            f'{self.controller_manager}/switch_controller',
            callback_group=self.callback_group,
        )
        self.list_controllers_client = self.create_client(
            ListControllers,
            f'{self.controller_manager}/list_controllers',
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
        self.create_subscription(
            JointState,
            args.joint_states_topic,
            self.joint_state_callback,
            rclpy.qos.qos_profile_sensor_data,
            callback_group=self.callback_group,
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
            f'Proxy ready: {self.proxy_action} -> {self.real_action}; '
            f'controller_manager={self.controller_manager}; '
            f'{self.velocity_controller} <-> {self.trajectory_controller}; '
            f'joint_states={args.joint_states_topic}'
        )

    def joint_state_callback(self, msg):
        with self._joint_lock:
            for index, name in enumerate(msg.name):
                if index < len(msg.position):
                    self._joint_positions[name] = msg.position[index]

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

    def _controller_states(self):
        if not self.list_controllers_client.wait_for_service(timeout_sec=0.5):
            return {}

        future = self.list_controllers_client.call_async(ListControllers.Request())
        if not self._wait_for_future(future, 1.0):
            return {}

        response = future.result()
        if response is None:
            return {}
        return {controller.name: controller.state for controller in response.controller}

    def _switch_controllers(self, activate, deactivate, reason):
        if not self.switch_client.wait_for_service(timeout_sec=self.args.switch_timeout):
            self.get_logger().error(
                f'Controller manager service not available: '
                f'{self.controller_manager}/switch_controller'
            )
            return False

        states = self._controller_states()
        if states:
            activate = [
                controller
                for controller in activate
                if states.get(controller) != 'active'
            ]
            deactivate = [
                controller
                for controller in deactivate
                if states.get(controller) == 'active'
            ]

        if not activate and not deactivate:
            self.get_logger().info(f'{reason}: controller state already correct')
            return True

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
        msg.data = [0.0] * ARM_JOINT_COUNT
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
        time.sleep(max(0.0, self.args.post_result_settle_sec))
        ok = self._switch_controllers(
            activate=[self.velocity_controller],
            deactivate=[self.trajectory_controller],
            reason='MoveIt execution finished',
        )
        if ok:
            self._publish_zero_velocity()
        return ok

    def _final_goal_positions(self, goal):
        trajectory = goal.trajectory
        if not trajectory.points:
            return {}
        for point in reversed(trajectory.points):
            if len(point.positions) >= len(trajectory.joint_names):
                return {
                    joint_name: point.positions[index]
                    for index, joint_name in enumerate(trajectory.joint_names)
                }
        return {}

    def _final_goal_error(self, goal):
        final_positions = self._final_goal_positions(goal)
        if not final_positions:
            return None
        with self._joint_lock:
            current_positions = dict(self._joint_positions)
        errors = [
            abs(current_positions[joint_name] - target)
            for joint_name, target in final_positions.items()
            if joint_name in current_positions
        ]
        if len(errors) != len(final_positions):
            return None
        return max(errors) if errors else None

    def _trajectory_summary(self, goal):
        trajectory = goal.trajectory
        if not trajectory.points:
            return "empty trajectory"
        duration = trajectory.points[-1].time_from_start
        seconds = duration.sec + duration.nanosec / 1_000_000_000.0
        return (
            f"joints={list(trajectory.joint_names)}, "
            f"points={len(trajectory.points)}, duration={seconds:.3f}s"
        )

    def _looks_like_deactivate_cancel(self, result):
        error_string = (result.error_string or '').lower()
        return (
            result.error_code == FollowJointTrajectory.Result.INVALID_GOAL
            and 'deactivate transition' in error_string
        )

    def execute_callback(self, goal_handle):
        result = FollowJointTrajectory.Result()
        self.get_logger().info(
            f"Accepted proxy goal: {self._trajectory_summary(goal_handle.request)}"
        )
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
            final_error = self._final_goal_error(goal_handle.request)
            self.get_logger().info(
                "Real trajectory result: "
                f"status={real_result.status}, "
                f"error_code={result.error_code}, "
                f"error_string='{result.error_string}', "
                f"final_error={final_error}"
            )
            if real_result.status == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
            elif self._looks_like_deactivate_cancel(result):
                if final_error is not None and final_error <= self.args.goal_reached_tolerance:
                    self.get_logger().warn(
                        'Real controller reported deactivate-transition cancel, '
                        f'but final joint error is {final_error:.5f} rad; reporting success.'
                    )
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = ''
                    goal_handle.succeed()
                else:
                    self.get_logger().warn(
                        'Deactivate-transition cancel with unresolved final error: '
                        f'{final_error}'
                    )
                    goal_handle.abort()
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
    parser.add_argument('--robot-name', default='mur620')
    parser.add_argument('--arm', choices=['l', 'r'], required=True)
    parser.add_argument('--proxy-action', default='')
    parser.add_argument('--real-action', default='')
    parser.add_argument('--controller-manager', default='')
    parser.add_argument('--velocity-controller', default='forward_velocity_controller')
    parser.add_argument('--trajectory-controller', default='joint_trajectory_controller')
    parser.add_argument('--velocity-command-topic', default='')
    parser.add_argument('--joint-states-topic', default='/joint_states')
    parser.add_argument('--switch-timeout', type=float, default=5.0)
    parser.add_argument('--action-timeout', type=float, default=10.0)
    parser.add_argument('--post-result-settle-sec', type=float, default=0.25)
    parser.add_argument('--goal-reached-tolerance', type=float, default=0.025)
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
