#!/usr/bin/env python3
"""Forward simple PoseStamped goals to the J-PARSE move action server."""

import argparse

import rclpy
from geometry_msgs.msg import PoseStamped
from mur_launch_sim.action import JparseMove
from rclpy.action import ActionClient
from rclpy.node import Node


class JparseSimpleGoal(Node):
    def __init__(self, args):
        super().__init__(f'{args.robot_name}_jparse_simple_goal_{args.arm}')
        self.args = args
        self.robot_name = args.robot_name
        self.arm = args.arm
        self.base_link = args.base_link or (
            'UR10_l/base_link' if args.arm == 'l' else 'UR10_r/base_link'
        )
        self.default_frame = f'{self.robot_name}/{self.base_link}'
        self.action_name = args.action_name or f'/{self.robot_name}/jparse_move_{self.arm}'
        self.approach_topic = args.approach_topic or (
            f'/{self.robot_name}/jparse_move_{self.arm}/simple_goal/approach'
        )
        self.precision_topic = args.precision_topic or (
            f'/{self.robot_name}/jparse_move_{self.arm}/simple_goal/precision'
        )
        self.precise_alias_topic = (
            f'/{self.robot_name}/jparse_move_{self.arm}/simple_goal/precise'
        )
        self.action_client = ActionClient(self, JparseMove, self.action_name)
        self.active_goal_handle = None

        self.create_subscription(PoseStamped, self.approach_topic, self.approach_callback, 10)
        self.create_subscription(PoseStamped, self.precision_topic, self.precision_callback, 10)
        self.create_subscription(PoseStamped, self.precise_alias_topic, self.precision_callback, 10)
        self.get_logger().info(
            f'J-PARSE simple goal bridge ready: {self.approach_topic} -> approach, '
            f'{self.precision_topic}/{self.precise_alias_topic} -> precision, '
            f'action={self.action_name}'
        )

    def approach_callback(self, msg):
        self.send_pose_goal(msg, 'approach')

    def precision_callback(self, msg):
        self.send_pose_goal(msg, 'precision')

    def send_pose_goal(self, pose_msg, accuracy):
        if not self.action_client.wait_for_server(timeout_sec=self.args.server_timeout):
            self.get_logger().error(f'Action server not available: {self.action_name}')
            return

        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None

        goal = JparseMove.Goal()
        goal.mode = 'task_space'
        goal.accuracy = accuracy
        goal.target_pose = pose_msg
        if not goal.target_pose.header.frame_id:
            goal.target_pose.header.frame_id = self.default_frame
        goal.max_linear_velocity = self.args.max_linear_velocity
        goal.max_angular_velocity = self.args.max_angular_velocity
        goal.timeout = self.args.timeout

        self.get_logger().info(
            f'Sending {accuracy} task-space pose goal in frame '
            f'{goal.target_pose.header.frame_id}'
        )
        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self.feedback_callback(accuracy, feedback),
        )
        future.add_done_callback(lambda done: self.goal_response_callback(accuracy, done))

    def goal_response_callback(self, accuracy, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'{accuracy} pose goal rejected')
            return
        self.active_goal_handle = goal_handle
        self.get_logger().info(f'{accuracy} pose goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done: self.result_callback(accuracy, done))

    def feedback_callback(self, accuracy, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'{accuracy} progress={feedback.progress:.2f}, '
            f'pos_err={feedback.position_error:.4f}, '
            f'ori_err={feedback.orientation_error:.4f}'
        )

    def result_callback(self, accuracy, future):
        result = future.result().result
        self.active_goal_handle = None
        level = self.get_logger().info if result.success else self.get_logger().warn
        level(
            f'{accuracy} pose goal finished: success={result.success}, '
            f'message="{result.message}", '
            f'pos_err={result.final_position_error:.4f}, '
            f'ori_err={result.final_orientation_error:.4f}'
        )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', default='mur620a')
    parser.add_argument('--arm', choices=['l', 'r'], default='l')
    parser.add_argument('--base-link', default='')
    parser.add_argument('--action-name', default='')
    parser.add_argument('--approach-topic', default='')
    parser.add_argument('--precision-topic', default='')
    parser.add_argument('--max-linear-velocity', type=float, default=0.12)
    parser.add_argument('--max-angular-velocity', type=float, default=0.5)
    parser.add_argument('--timeout', type=float, default=20.0)
    parser.add_argument('--server-timeout', type=float, default=0.2)
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = JparseSimpleGoal(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
