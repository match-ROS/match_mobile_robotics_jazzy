#!/usr/bin/env python3
"""Publish MoveIt robot descriptions as latched topics for RViz clients."""

import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import xacro


class MoveItDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('moveit_description_publisher')
        self.declare_parameter('robot_name', 'mur620a')
        self.declare_parameter('publish_global_topics', True)
        self.declare_parameter('publish_namespaced_topics', True)
        self.declare_parameter('srdf_package', 'mur_moveit_config')
        self.declare_parameter('srdf_path', 'srdf/mur620.srdf.xacro')
        self.declare_parameter('srdf_prefix', 'UR10')
        self.declare_parameter('srdf_model_name', 'mur620')

        robot_name = self.get_parameter('robot_name').value.strip('/')
        publish_global = self.get_parameter('publish_global_topics').value
        publish_namespaced = self.get_parameter('publish_namespaced_topics').value

        srdf = self.load_srdf()
        topics = []
        if publish_global:
            topics.append('/robot_description_semantic')
        if publish_namespaced and robot_name:
            topics.append(f'/{robot_name}/robot_description_semantic')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.description_publishers = [
            self.create_publisher(String, topic, qos) for topic in topics
        ]
        self.message = String(data=srdf)
        self.timer = self.create_timer(1.0, self.publish_descriptions)
        self.publish_descriptions()
        self.get_logger().info(
            'Publishing robot_description_semantic on: ' + ', '.join(topics)
        )

    def load_srdf(self):
        package_name = self.get_parameter('srdf_package').value
        relative_path = self.get_parameter('srdf_path').value
        srdf_path = os.path.join(get_package_share_directory(package_name), relative_path)
        mappings = {
            'prefix': self.get_parameter('srdf_prefix').value,
            'model_name': self.get_parameter('srdf_model_name').value,
        }
        return xacro.process_file(srdf_path, mappings=mappings).toxml()

    def publish_descriptions(self):
        for publisher in self.description_publishers:
            publisher.publish(self.message)


def main():
    rclpy.init()
    node = MoveItDescriptionPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
