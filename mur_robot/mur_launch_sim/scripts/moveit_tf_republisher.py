#!/usr/bin/env python3
"""Republish one robot's TF tree on private MoveIt TF topics.

The MUR620 MoveIt model intentionally keeps unprefixed link names such as
``base_footprint`` and ``UR10_l/tool0``. That works for one robot if an alias
from ``<robot>/base_footprint`` to ``base_footprint`` is available, but it
cannot work for several robots on the shared /tf graph. This node creates a
per-robot TF view for MoveIt/RViz so each instance can have its own local
``base_footprint`` without colliding globally.
"""

import argparse
import copy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def is_static_topic(topic_name):
    return topic_name.endswith("tf_static")


class MoveItTfRepublisher(Node):
    def __init__(self, robot_name, tf_topic, tf_static_topic):
        super().__init__(f"{robot_name}_moveit_tf_republisher")
        self.robot_name = robot_name
        self.robot_prefix = f"{robot_name}/"
        self.robot_odom = f"{robot_name}/odom"
        self.robot_base = f"{robot_name}/base_footprint"
        self.static_transforms = {}

        dynamic_qos = QoSProfile(depth=100)
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.tf_pub = self.create_publisher(TFMessage, tf_topic, dynamic_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, tf_static_topic, static_qos)
        self.create_subscription(TFMessage, "/tf", self.dynamic_cb, dynamic_qos)
        self.create_subscription(TFMessage, "/tf_static", self.static_cb, static_qos)

        self.publish_static_alias()
        self.create_timer(1.0, self.publish_static_snapshot)

        self.get_logger().info(
            f"Republishing {robot_name} TF for MoveIt on {tf_topic} and {tf_static_topic}"
        )

    def should_keep_external_base_transform(self, transform):
        parent = transform.header.frame_id
        child = transform.child_frame_id
        return (
            (parent in ("map", "world") and child in (self.robot_odom, self.robot_base))
            or (parent == self.robot_odom and child == self.robot_base)
        )

    def rewrite_frame(self, frame_id):
        if frame_id.startswith(self.robot_prefix):
            return frame_id[len(self.robot_prefix):]
        return frame_id

    def rewrite_transform(self, transform):
        parent = transform.header.frame_id
        child = transform.child_frame_id

        if not (parent.startswith(self.robot_prefix) or child.startswith(self.robot_prefix)):
            return None

        transformed = copy.deepcopy(transform)
        if self.should_keep_external_base_transform(transformed):
            return transformed

        transformed.header.frame_id = self.rewrite_frame(parent)
        transformed.child_frame_id = self.rewrite_frame(child)
        if transformed.header.frame_id == transformed.child_frame_id:
            return None
        return transformed

    def dynamic_cb(self, msg):
        transforms = [
            transformed
            for transform in msg.transforms
            if (transformed := self.rewrite_transform(transform)) is not None
        ]
        if transforms:
            self.tf_pub.publish(TFMessage(transforms=transforms))

    def static_cb(self, msg):
        changed = False
        for transform in msg.transforms:
            transformed = self.rewrite_transform(transform)
            if transformed is None:
                continue
            key = (transformed.header.frame_id, transformed.child_frame_id)
            self.static_transforms[key] = transformed
            changed = True

        if changed:
            self.publish_static_snapshot()

    def publish_static_alias(self):
        from geometry_msgs.msg import TransformStamped

        alias = TransformStamped()
        alias.header.stamp = self.get_clock().now().to_msg()
        alias.header.frame_id = self.robot_base
        alias.child_frame_id = "base_footprint"
        alias.transform.rotation.w = 1.0
        self.static_transforms[(alias.header.frame_id, alias.child_frame_id)] = alias
        self.publish_static_snapshot()

    def publish_static_snapshot(self):
        if not self.static_transforms:
            return
        now = self.get_clock().now().to_msg()
        transforms = []
        for transform in self.static_transforms.values():
            updated = copy.deepcopy(transform)
            updated.header.stamp = now
            transforms.append(updated)
        self.tf_static_pub.publish(TFMessage(transforms=transforms))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-name", required=True)
    parser.add_argument("--tf-topic", required=True)
    parser.add_argument("--tf-static-topic", required=True)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = MoveItTfRepublisher(args.robot_name, args.tf_topic, args.tf_static_topic)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
