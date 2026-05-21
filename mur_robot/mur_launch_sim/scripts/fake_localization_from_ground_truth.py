#!/usr/bin/env python3
"""Publish a lightweight fake localization TF from Gazebo ground truth."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def normalize_quaternion(q):
    norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def transform_from_odom(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)


class FakeLocalizationFromGroundTruth(Node):
    def __init__(self):
        super().__init__("fake_localization_from_ground_truth")

        self.declare_parameter("robot_name", "mur620a")
        self.declare_parameter("global_frame_id", "map")
        self.declare_parameter("odom_frame_id", "")
        self.declare_parameter("base_frame_id", "")
        self.declare_parameter("ground_truth_odom_topic", "")
        self.declare_parameter("publish_rate", 30.0)

        self.robot_name = self.get_parameter("robot_name").value
        self.global_frame_id = self.get_parameter("global_frame_id").value
        self.odom_frame_id = (
            self.get_parameter("odom_frame_id").value or f"{self.robot_name}/odom"
        )
        self.base_frame_id = (
            self.get_parameter("base_frame_id").value
            or f"{self.robot_name}/base_footprint"
        )
        ground_truth_odom_topic = (
            self.get_parameter("ground_truth_odom_topic").value
            or f"/{self.robot_name}/ground_truth/odom"
        )
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.publish_period = 1.0 / max(publish_rate, 1.0)
        self.last_publish_time = None
        self.logged_ready = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(Odometry, ground_truth_odom_topic, self.ground_truth_cb, 10)
        self.publish_odom_to_base_identity()

        self.get_logger().info(
            f"Fake localization for {self.robot_name}: "
            f"{ground_truth_odom_topic} -> {self.global_frame_id} -> "
            f"{self.odom_frame_id} -> {self.base_frame_id}"
        )

    def ground_truth_cb(self, msg):
        stamp_time = Time.from_msg(msg.header.stamp)
        stamp_sec = stamp_time.nanoseconds * 1e-9
        if (
            self.last_publish_time is not None
            and stamp_sec > self.last_publish_time
            and stamp_sec - self.last_publish_time < self.publish_period
        ):
            return

        self.last_publish_time = stamp_sec
        self.publish_tf(msg)

    def publish_tf(self, ground_truth_msg):
        map_to_odom = transform_from_odom(ground_truth_msg)

        transform_msg = TransformStamped()
        transform_msg.header.stamp = ground_truth_msg.header.stamp
        transform_msg.header.frame_id = self.global_frame_id
        transform_msg.child_frame_id = self.odom_frame_id
        transform_msg.transform.translation.x = map_to_odom[0][0]
        transform_msg.transform.translation.y = map_to_odom[0][1]
        transform_msg.transform.translation.z = map_to_odom[0][2]
        transform_msg.transform.rotation.x = map_to_odom[1][0]
        transform_msg.transform.rotation.y = map_to_odom[1][1]
        transform_msg.transform.rotation.z = map_to_odom[1][2]
        transform_msg.transform.rotation.w = map_to_odom[1][3]
        self.tf_broadcaster.sendTransform(transform_msg)

        if not self.logged_ready:
            self.logged_ready = True
            self.get_logger().info(
                f"Publishing fake localization TF {self.global_frame_id} -> "
                f"{self.odom_frame_id}"
            )

    def publish_odom_to_base_identity(self):
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = self.odom_frame_id
        transform_msg.child_frame_id = self.base_frame_id
        transform_msg.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform_msg)
        self.get_logger().info(
            f"Publishing static fake odom TF {self.odom_frame_id} -> "
            f"{self.base_frame_id}"
        )


def main():
    rclpy.init()
    node = FakeLocalizationFromGroundTruth()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
