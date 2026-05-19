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
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINT_COUNT = 6


class MoveItTrajectoryControllerProxy(Node):
    def __init__(self, args):
        node_suffix = f'lift_{args.arm}' if args.include_lift else args.arm
        super().__init__(f'moveit_trajectory_controller_proxy_{node_suffix}')
        self.args = args
        self.callback_group = ReentrantCallbackGroup()
        self._active_client_goal = None
        self._client_goal_lock = threading.Lock()
        self._last_lift_position = None

        proxy_suffix = f'lift_{args.arm}' if args.include_lift else args.arm

        self.proxy_action = (
            f'/{args.robot_name}/moveit_joint_trajectory_controller_{proxy_suffix}/'
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
        self.lift_controller = f'lift_controller_{args.arm}'
        self.lift_joint = 'left_lift_joint' if args.arm == 'l' else 'right_lift_joint'
        self.lift_command_topic = f'/{args.robot_name}/{self.lift_controller}/commands'

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
        self.lift_position_pub = self.create_publisher(
            Float64MultiArray,
            self.lift_command_topic,
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
        msg.data = [0.0] * ARM_JOINT_COUNT
        for _ in range(3):
            self.zero_velocity_pub.publish(msg)
            time.sleep(0.02)

    def _publish_lift_position(self, position):
        if not self.args.include_lift or position is None:
            return
        msg = Float64MultiArray()
        msg.data = [float(position)]
        for _ in range(3):
            self.lift_position_pub.publish(msg)
            time.sleep(0.02)

    def _final_lift_position(self, trajectory):
        if not self.args.include_lift or self.lift_joint not in trajectory.joint_names:
            return None
        lift_index = trajectory.joint_names.index(self.lift_joint)
        for point in reversed(trajectory.points):
            if len(point.positions) > lift_index:
                return point.positions[lift_index]
        return None

    def _seconds(self, duration):
        return duration.sec + duration.nanosec / 1_000_000_000

    def _filter_values(self, values, indices):
        if not values:
            return []
        return [values[index] for index in indices if len(values) > index]

    def _filter_tolerances(self, tolerances, joint_names):
        joint_name_set = set(joint_names)
        return [
            tolerance
            for tolerance in tolerances
            if not tolerance.name or tolerance.name in joint_name_set
        ]

    def _arm_goal(self, request):
        trajectory = request.trajectory
        arm_indices = [
            index
            for index, joint_name in enumerate(trajectory.joint_names)
            if joint_name != self.lift_joint
        ]
        arm_joint_names = [
            trajectory.joint_names[index]
            for index in arm_indices
        ]

        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.header = trajectory.header
        arm_goal.trajectory.joint_names = arm_joint_names
        arm_goal.multi_dof_trajectory = request.multi_dof_trajectory
        arm_goal.path_tolerance = self._filter_tolerances(
            request.path_tolerance,
            arm_joint_names,
        )
        arm_goal.goal_tolerance = self._filter_tolerances(
            request.goal_tolerance,
            arm_joint_names,
        )
        arm_goal.goal_time_tolerance = request.goal_time_tolerance

        for point in trajectory.points:
            arm_point = JointTrajectoryPoint()
            arm_point.positions = self._filter_values(point.positions, arm_indices)
            arm_point.velocities = self._filter_values(point.velocities, arm_indices)
            arm_point.accelerations = self._filter_values(
                point.accelerations,
                arm_indices,
            )
            arm_point.effort = self._filter_values(point.effort, arm_indices)
            arm_point.time_from_start = point.time_from_start
            arm_goal.trajectory.points.append(arm_point)

        return arm_goal

    def _lift_points(self, trajectory):
        if not self.args.include_lift or self.lift_joint not in trajectory.joint_names:
            return []
        lift_index = trajectory.joint_names.index(self.lift_joint)
        points = []
        for point in trajectory.points:
            if len(point.positions) > lift_index:
                points.append((self._seconds(point.time_from_start), point.positions[lift_index]))
        return points

    def _publish_lift_trajectory(self, points, stop_event):
        start_time = time.monotonic()
        last_position = None
        for target_time, position in points:
            while rclpy.ok() and not stop_event.is_set():
                if time.monotonic() - start_time >= target_time:
                    break
                time.sleep(0.01)
            if stop_event.is_set():
                return
            self._publish_lift_position(position)
            last_position = position

        if last_position is not None:
            self._publish_lift_position(last_position)

    def _switch_to_trajectory(self):
        self._publish_zero_velocity()
        return self._switch_controllers(
            activate=[self.trajectory_controller],
            deactivate=[self.velocity_controller],
            reason='MoveIt execution start',
        )

    def _switch_to_velocity(self, lift_position=None):
        ok = self._switch_controllers(
            activate=[self.velocity_controller],
            deactivate=[self.trajectory_controller],
            reason='MoveIt execution finished',
        )
        if ok:
            self._publish_zero_velocity()
            self._publish_lift_position(lift_position)
        return ok

    def execute_callback(self, goal_handle):
        result = FollowJointTrajectory.Result()
        self._last_lift_position = self._final_lift_position(
            goal_handle.request.trajectory
        )
        forwarded_goal = (
            self._arm_goal(goal_handle.request)
            if self.args.include_lift
            else goal_handle.request
        )
        lift_points = self._lift_points(goal_handle.request.trajectory)
        lift_stop_event = threading.Event()
        lift_thread = None

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
                forwarded_goal,
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

            if lift_points:
                lift_thread = threading.Thread(
                    target=self._publish_lift_trajectory,
                    args=(lift_points, lift_stop_event),
                    daemon=True,
                )
                lift_thread.start()

            result_future = client_goal.get_result_async()
            while rclpy.ok() and not result_future.done():
                if goal_handle.is_cancel_requested:
                    client_goal.cancel_goal_async()
                    lift_stop_event.set()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = 'Trajectory execution canceled'
                    return result
                time.sleep(0.02)

            if lift_thread is not None:
                lift_thread.join(timeout=self.args.action_timeout)

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
            lift_stop_event.set()
            with self._client_goal_lock:
                self._active_client_goal = None
            self._switch_to_velocity(self._last_lift_position)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620a')
    parser.add_argument('--arm', choices=['l', 'r'], required=True)
    parser.add_argument('--include-lift', action='store_true')
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
