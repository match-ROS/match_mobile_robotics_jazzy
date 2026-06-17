#!/usr/bin/env python3
from math import atan2, cos, sin

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    class Quaternion:
        pass

    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = sin(yaw / 2.0)
    q.w = cos(yaw / 2.0)
    return q


class ExternalLocalizationBroadcaster(Node):
    def __init__(self):
        super().__init__('external_localization_broadcaster')
        self.tf_prefix = self.declare_parameter('tf_prefix', '').value.strip('/')
        self.localization_topic = self.declare_parameter('localization_topic', '/qualisys/mur620d/pose').value
        self.mocap_offset = list(self.declare_parameter('mocap_offset', [0.0, 0.0, 0.0]).value)
        while len(self.mocap_offset) < 3:
            self.mocap_offset.append(0.0)
        self.broadcaster = TransformBroadcaster(self)
        self.last_stamp_ns = 0
        self.subscription = self.create_subscription(PoseStamped, self.localization_topic, self.callback, 10)

    def callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= self.last_stamp_ns:
            now_ns = self.last_stamp_ns + 1000000
        self.last_stamp_ns = now_ns

        pose = self.transform_pose_to_mir_map(msg.pose)
        transform = TransformStamped()
        transform.header.stamp = Time(nanoseconds=now_ns).to_msg()
        transform.header.frame_id = self._frame('odom')
        transform.child_frame_id = self._frame('base_footprint')
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self.broadcaster.sendTransform(transform)

    def transform_pose_to_mir_map(self, pose):
        pose_out = Pose()
        offset_x, offset_y, offset_yaw = self.mocap_offset[:3]
        pose_out.position.x = offset_x + pose.position.x * cos(offset_yaw) - pose.position.y * sin(offset_yaw)
        pose_out.position.y = offset_y + pose.position.x * sin(offset_yaw) + pose.position.y * cos(offset_yaw)
        pose_out.position.z = pose.position.z
        pose_out.orientation = yaw_to_quaternion(offset_yaw + quaternion_to_yaw(pose.orientation))
        return pose_out

    def _frame(self, frame):
        return '{}/{}'.format(self.tf_prefix, frame) if self.tf_prefix else frame


def main(args=None):
    rclpy.init(args=args)
    node = ExternalLocalizationBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
