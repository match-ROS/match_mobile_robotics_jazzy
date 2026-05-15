#!/usr/bin/env python3
"""Publish robot ground truth pose/odom from Gazebo world pose messages."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GroundTruthFromPoseTf(Node):
    def __init__(self):
        super().__init__('ground_truth_from_pose_tf')

        self.declare_parameter('input_topic', '')
        self.declare_parameter('robot_name', 'mur620a')
        self.declare_parameter('output_frame_id', 'map')
        self.declare_parameter('child_frame_id', '')
        self.declare_parameter('pose_topic', '')
        self.declare_parameter('odom_topic', '')

        self.input_topic = self.get_parameter('input_topic').value
        self.robot_name = self.get_parameter('robot_name').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        child_frame_id = self.get_parameter('child_frame_id').value
        self.child_frame_id = child_frame_id or f'{self.robot_name}/base_footprint'
        pose_topic = self.get_parameter('pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        if not self.input_topic:
            raise ValueError('input_topic must be set')
        if not pose_topic:
            pose_topic = f'/{self.robot_name}/ground_truth/pose'
        if not odom_topic:
            odom_topic = f'/{self.robot_name}/ground_truth/odom'

        self.target_names = {
            self.robot_name,
            f'{self.robot_name}/base_link',
            f'{self.robot_name}/base_footprint',
            f'{self.robot_name}::base_link',
            f'{self.robot_name}::base_footprint',
            'base_link',
            'base_footprint',
        }
        self.logged_available_names = False
        self.previous_stamp = None
        self.previous_transform = None

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.subscription = self.create_subscription(
            TFMessage,
            self.input_topic,
            self.pose_callback,
            10,
        )

        self.get_logger().info(
            f'Publishing ground truth from {self.input_topic} for {sorted(self.target_names)}'
        )

    def pose_callback(self, msg: TFMessage):
        transform = self.find_robot_transform(msg)
        if transform is None:
            self.log_available_names_once(msg)
            return

        pose_msg = self.make_pose(transform)
        odom_msg = self.make_odom(transform)

        self.pose_pub.publish(pose_msg)
        self.odom_pub.publish(odom_msg)

        self.previous_stamp = transform.header.stamp
        self.previous_transform = transform

    def find_robot_transform(self, msg: TFMessage):
        for transform in msg.transforms:
            if self.is_target_transform(transform):
                return transform
        return None

    def is_target_transform(self, transform: TransformStamped) -> bool:
        names = [
            transform.child_frame_id.strip('/'),
            transform.header.frame_id.strip('/'),
        ]
        for name in names:
            if not name:
                continue
            if name in self.target_names:
                return True
            if name == self.robot_name:
                return True
            if name.endswith(f'{self.robot_name}/base_footprint'):
                return True
            if name.endswith(f'{self.robot_name}/base_link'):
                return True
            if name.endswith(f'{self.robot_name}::base_footprint'):
                return True
            if name.endswith(f'{self.robot_name}::base_link'):
                return True
        return False

    def log_available_names_once(self, msg: TFMessage):
        if self.logged_available_names:
            return
        self.logged_available_names = True
        names = []
        for transform in msg.transforms:
            child_name = transform.child_frame_id.strip('/')
            parent_name = transform.header.frame_id.strip('/')
            if child_name:
                names.append(f'child={child_name}')
            if parent_name:
                names.append(f'parent={parent_name}')
        preview = ', '.join(names[:40])
        if len(names) > 40:
            preview += ', ...'
        self.get_logger().warning(
            f'No ground truth match for {sorted(self.target_names)}. '
            f'Available Gazebo pose names: {preview}'
        )

    def make_pose(self, transform: TransformStamped) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = transform.header.stamp
        pose_msg.header.frame_id = self.output_frame_id
        pose_msg.pose.position.x = transform.transform.translation.x
        pose_msg.pose.position.y = transform.transform.translation.y
        pose_msg.pose.position.z = transform.transform.translation.z
        pose_msg.pose.orientation = transform.transform.rotation
        return pose_msg

    def make_odom(self, transform: TransformStamped) -> Odometry:
        odom_msg = Odometry()
        odom_msg.header.stamp = transform.header.stamp
        odom_msg.header.frame_id = self.output_frame_id
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose = self.make_pose(transform).pose
        odom_msg.twist.twist = self.estimate_twist(transform)
        return odom_msg

    def estimate_twist(self, transform: TransformStamped) -> Twist:
        twist = Twist()
        if self.previous_stamp is None or self.previous_transform is None:
            return twist

        now = self.stamp_to_sec(transform.header.stamp)
        previous = self.stamp_to_sec(self.previous_stamp)
        dt = now - previous
        if dt <= 0.0:
            return twist

        translation = transform.transform.translation
        previous_translation = self.previous_transform.transform.translation
        twist.linear.x = (translation.x - previous_translation.x) / dt
        twist.linear.y = (translation.y - previous_translation.y) / dt
        twist.linear.z = (translation.z - previous_translation.z) / dt

        yaw = yaw_from_quaternion(transform.transform.rotation)
        previous_yaw = yaw_from_quaternion(self.previous_transform.transform.rotation)
        twist.angular.z = normalize_angle(yaw - previous_yaw) / dt
        return twist

    @staticmethod
    def stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main():
    rclpy.init()
    node = GroundTruthFromPoseTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
