#!/usr/bin/env python3
"""Pose action server that drives the integrated Cartesian controller."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mur_control.action import JparseMove
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def clamp_norm(values, max_norm):
    norm = math.sqrt(sum(value * value for value in values))
    if max_norm <= 0.0 or norm <= max_norm or norm <= 1.0e-12:
        return values
    scale = max_norm / norm
    return [value * scale for value in values]


def quat_normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1.0e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [value / norm for value in q]


def quat_conjugate(q):
    return [-q[0], -q[1], -q[2], q[3]]


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def quat_slerp(q0, q1, ratio):
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = [-value for value in q1]
        dot = -dot
    dot = clamp(dot, -1.0, 1.0)
    if dot > 0.9995:
        return quat_normalize([
            q0[index] + ratio * (q1[index] - q0[index])
            for index in range(4)
        ])
    theta_0 = math.acos(dot)
    theta = theta_0 * ratio
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return [s0 * q0[index] + s1 * q1[index] for index in range(4)]


def quat_error_vector(target, current):
    error = quat_multiply(target, quat_conjugate(current))
    error = quat_normalize(error)
    if error[3] < 0.0:
        error = [-value for value in error]
    vector_norm = math.sqrt(error[0] ** 2 + error[1] ** 2 + error[2] ** 2)
    if vector_norm <= 1.0e-12:
        return [0.0, 0.0, 0.0], 0.0
    angle = 2.0 * math.atan2(vector_norm, error[3])
    axis = [error[index] / vector_norm for index in range(3)]
    return [axis[index] * angle for index in range(3)], abs(angle)


def quintic_profile(t, duration):
    if duration <= 1.0e-9:
        return 1.0, 0.0
    tau = clamp(t / duration, 0.0, 1.0)
    pos = 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5
    vel = (30.0 * tau ** 2 - 60.0 * tau ** 3 + 30.0 * tau ** 4) / duration
    return pos, vel


class Pid3:
    def __init__(self, kp, ki, kd, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.integral = [0.0, 0.0, 0.0]
        self.previous = None

    def update(self, error, dt):
        dt = max(dt, 1.0e-6)
        derivative = [0.0, 0.0, 0.0] if self.previous is None else [
            (error[index] - self.previous[index]) / dt for index in range(3)
        ]
        self.previous = list(error)
        for index in range(3):
            self.integral[index] = clamp(
                self.integral[index] + error[index] * dt,
                -self.integral_limit,
                self.integral_limit,
            )
        return [
            self.kp * error[index] + self.ki * self.integral[index] + self.kd * derivative[index]
            for index in range(3)
        ]


class IntegratedCartesianMoveActionServer(Node):
    def __init__(self, args):
        super().__init__(f'{args.robot_name}_integrated_cartesian_move_{args.arm}')
        self.args = args
        self.robot_name = args.robot_name
        self.arm = args.arm
        self.base_link = args.base_link or (
            'UR10_l/base_link' if args.arm == 'l' else 'UR10_r/base_link'
        )
        self.tip_link = args.tip_link or (
            'UR10_l/tool0' if args.arm == 'l' else 'UR10_r/tool0'
        )
        self.tf_base_frame = f'{self.robot_name}/{self.base_link}'
        self.tf_tip_frame = f'{self.robot_name}/{self.tip_link}'
        self.default_action_name = f'/{self.robot_name}/jparse_move_{self.arm}'
        self.action_name = args.action_name or self.default_action_name
        self.twist_topic = args.twist_topic or (
            f'/{self.robot_name}/UR10_{self.arm}/'
            'integrated_cartesian_admittance_controller/equilibrium_twist_cmd'
        )
        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_server = ActionServer(
            self,
            JparseMove,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(
            f'Integrated Cartesian move action ready on {self.action_name}; '
            f'modes: task_space; twist={self.twist_topic}; '
            f'tf={self.tf_base_frame}->{self.tf_tip_frame}'
        )

    def goal_callback(self, goal_request):
        mode = goal_request.mode.strip().lower()
        accuracy = goal_request.accuracy.strip().lower()
        if mode != 'task_space':
            self.get_logger().warn(
                f'Rejecting goal with mode {goal_request.mode}: '
                'integrated Cartesian move supports task_space only'
            )
            return GoalResponse.REJECT
        if accuracy and accuracy not in ('approach', 'precision'):
            self.get_logger().warn(f'Rejecting goal with unknown accuracy: {goal_request.accuracy}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def get_profile(self, accuracy):
        if accuracy == 'precision':
            return {
                'position_tolerance': 0.003,
                'orientation_tolerance': math.radians(0.5),
                'settle_time': 1.0,
                'kp_linear': 1.6,
                'ki_linear': 0.03,
                'kd_linear': 0.02,
                'kp_angular': 1.4,
                'ki_angular': 0.02,
                'kd_angular': 0.01,
            }
        return {
            'position_tolerance': 0.025,
            'orientation_tolerance': math.radians(3.0),
            'settle_time': 0.25,
            'kp_linear': 1.0,
            'ki_linear': 0.0,
            'kd_linear': 0.01,
            'kp_angular': 1.0,
            'ki_angular': 0.0,
            'kd_angular': 0.005,
        }

    def lookup_pose(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        last_error = None
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.tf_base_frame,
                    self.tf_tip_frame,
                    rclpy.time.Time(),
                )
                trans = transform.transform.translation
                rot = transform.transform.rotation
                return [trans.x, trans.y, trans.z], quat_normalize([rot.x, rot.y, rot.z, rot.w])
            except TransformException as exc:
                last_error = exc
                time.sleep(0.02)
        raise RuntimeError(f'Could not lookup TF {self.tf_base_frame}->{self.tf_tip_frame}: {last_error}')

    def publish_twist(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_link
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = linear
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular
        self.twist_pub.publish(msg)

    def stop(self):
        self.publish_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    def pose_from_goal(self, goal):
        pose = goal.target_pose.pose
        frame = goal.target_pose.header.frame_id or self.tf_base_frame
        pos = [pose.position.x, pose.position.y, pose.position.z]
        quat = quat_normalize([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])
        if frame in (self.tf_base_frame, self.base_link):
            return pos, quat
        raise RuntimeError(
            f'target_pose frame "{frame}" is not supported yet; use {self.tf_base_frame}'
        )

    def estimate_duration(self, start_pos, start_quat, target_pos, target_quat, goal):
        max_linear = goal.max_linear_velocity if goal.max_linear_velocity > 0.0 else self.args.max_linear_velocity
        max_angular = goal.max_angular_velocity if goal.max_angular_velocity > 0.0 else self.args.max_angular_velocity
        distance = math.sqrt(sum((target_pos[index] - start_pos[index]) ** 2 for index in range(3)))
        _, angle = quat_error_vector(target_quat, start_quat)
        linear_time = distance / max(max_linear, 1.0e-6) * 1.875
        angular_time = angle / max(max_angular, 1.0e-6) * 1.875
        return clamp(max(linear_time, angular_time, 0.5), 0.5, goal.timeout if goal.timeout > 0.0 else 30.0)

    def execute_callback(self, goal_handle):
        try:
            return self.execute_task_space(goal_handle)
        except Exception as exc:
            self.stop()
            goal_handle.abort()
            result = JparseMove.Result()
            result.success = False
            result.message = str(exc)
            self.get_logger().error(result.message)
            return result

    def execute_task_space(self, goal_handle):
        goal = goal_handle.request
        accuracy = goal.accuracy.strip().lower() or 'approach'
        profile = self.get_profile(accuracy)
        start_pos, start_quat = self.lookup_pose()
        target_pos, target_quat = self.pose_from_goal(goal)
        duration = self.estimate_duration(start_pos, start_quat, target_pos, target_quat, goal)
        max_linear = goal.max_linear_velocity if goal.max_linear_velocity > 0.0 else self.args.max_linear_velocity
        max_angular = goal.max_angular_velocity if goal.max_angular_velocity > 0.0 else self.args.max_angular_velocity
        linear_pid = Pid3(profile['kp_linear'], profile['ki_linear'], profile['kd_linear'], 0.05)
        angular_pid = Pid3(profile['kp_angular'], profile['ki_angular'], profile['kd_angular'], 0.25)
        last_time = time.monotonic()
        start_time = last_time
        feedback = JparseMove.Feedback()
        settled_since = None

        self.get_logger().info(
            f'Move goal accepted: distance={math.sqrt(sum((target_pos[i] - start_pos[i]) ** 2 for i in range(3))):.4f}m, '
            f'duration={duration:.2f}s, accuracy={accuracy}, max_linear={max_linear:.3f}'
        )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop()
                goal_handle.canceled()
                return self.make_result(False, 'Canceled')
            now = time.monotonic()
            elapsed = now - start_time
            dt = now - last_time
            last_time = now
            s, sdot = quintic_profile(elapsed, duration)
            desired_pos = [
                start_pos[index] + (target_pos[index] - start_pos[index]) * s
                for index in range(3)
            ]
            desired_quat = quat_slerp(start_quat, target_quat, s)
            current_pos, current_quat = self.lookup_pose(timeout=0.2)
            pos_error = [desired_pos[index] - current_pos[index] for index in range(3)]
            orient_error, _ = quat_error_vector(desired_quat, current_quat)
            final_pos_error = [target_pos[index] - current_pos[index] for index in range(3)]
            final_orient_error, final_angle = quat_error_vector(target_quat, current_quat)
            ff_linear = [
                (target_pos[index] - start_pos[index]) * sdot
                for index in range(3)
            ]
            total_angle_vec, _ = quat_error_vector(target_quat, start_quat)
            ff_angular = [value * sdot for value in total_angle_vec]
            linear_correction = linear_pid.update(pos_error, dt)
            angular_correction = angular_pid.update(orient_error, dt)
            linear = [
                ff_linear[index] + linear_correction[index]
                for index in range(3)
            ]
            angular = [
                ff_angular[index] + angular_correction[index]
                for index in range(3)
            ]
            linear = clamp_norm(linear, max_linear)
            angular = clamp_norm(angular, max_angular)
            self.publish_twist(linear, angular)

            position_error_norm = math.sqrt(sum(value * value for value in final_pos_error))
            feedback.progress = clamp(elapsed / duration, 0.0, 1.0)
            feedback.position_error = position_error_norm
            feedback.orientation_error = final_angle
            feedback.joint_error = 0.0
            goal_handle.publish_feedback(feedback)

            if (
                elapsed >= duration and
                position_error_norm <= profile['position_tolerance'] and
                final_angle <= profile['orientation_tolerance']
            ):
                settled_since = settled_since or now
                if now - settled_since >= profile['settle_time']:
                    self.stop()
                    goal_handle.succeed()
                    return self.make_result(
                        True,
                        'Reached integrated Cartesian task-space target',
                        position_error_norm,
                        final_angle,
                    )
            else:
                settled_since = None

            timeout = goal.timeout if goal.timeout > 0.0 else duration + 10.0
            if elapsed > timeout:
                self.stop()
                goal_handle.abort()
                return self.make_result(
                    False,
                    'Integrated Cartesian task-space target timed out',
                    position_error_norm,
                    final_angle,
                )
            time.sleep(1.0 / self.args.control_rate)

    def make_result(self, success, message, position_error=0.0, orientation_error=0.0):
        result = JparseMove.Result()
        result.success = success
        result.message = message
        result.final_position_error = position_error
        result.final_orientation_error = orientation_error
        result.final_joint_error = 0.0
        return result


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620')
    parser.add_argument('--arm', choices=['l', 'r'], default='r')
    parser.add_argument('--action-name', default='')
    parser.add_argument('--base-link', default='')
    parser.add_argument('--tip-link', default='')
    parser.add_argument('--twist-topic', default='')
    parser.add_argument('--control-rate', type=float, default=100.0)
    parser.add_argument('--max-linear-velocity', type=float, default=0.10)
    parser.add_argument('--max-angular-velocity', type=float, default=0.35)
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = IntegratedCartesianMoveActionServer(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
