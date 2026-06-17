#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class MirPoseSimple(Node):
    def __init__(self):
        super().__init__('mir_pose_simple')
        self.localization_type = self.declare_parameter('localization_type', 'robot_pose').value
        self.odom_topic = self.declare_parameter('odom_topic', 'odom').value
        self.amcl_pose_topic = self.declare_parameter('amcl_pose_topic', 'amcl_pose').value
        self.ground_truth_topic = self.declare_parameter('ground_truth_topic', 'ground_truth').value
        self.mocap_topic = self.declare_parameter('mocap_topic', '/qualisys/mur620b/pose').value
        self.robot_pose_topic = self.declare_parameter('robot_pose_topic', 'robot_pose').value
        self.output_frame = self.declare_parameter('output_frame', 'map').value

        self.pose_pub = self.create_publisher(Pose, 'mir_pose_simple', 1)
        self.pose_stamped_pub = self.create_publisher(PoseStamped, 'mir_pose_stamped_simple', 1)
        self.subscriptions = []

        if self.localization_type == 'amcl':
            self.subscriptions.append(self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose_topic, self.amcl_pose_callback, 10))
        elif self.localization_type == 'ground_truth':
            self.subscriptions.append(self.create_subscription(Odometry, self.ground_truth_topic, self.odom_callback, 10))
        elif self.localization_type == 'odom':
            self.subscriptions.append(self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10))
        elif self.localization_type == 'mocap':
            self.subscriptions.append(self.create_subscription(PoseStamped, self.mocap_topic, self.mocap_callback, 10))
        elif self.localization_type == 'robot_pose':
            self.subscriptions.append(self.create_subscription(Pose, self.robot_pose_topic, self.robot_pose_callback, 10))
        else:
            self.get_logger().warning('Unknown localization_type: {}'.format(self.localization_type))

    def odom_callback(self, msg):
        self._publish(msg.pose.pose, msg.header)

    def amcl_pose_callback(self, msg):
        self._publish(msg.pose.pose, msg.header)

    def mocap_callback(self, msg):
        self._publish(msg.pose, msg.header)

    def robot_pose_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.output_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = msg
        self.pose_pub.publish(msg)
        self.pose_stamped_pub.publish(pose_stamped)

    def _publish(self, pose, header):
        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.pose = pose
        self.pose_pub.publish(pose)
        self.pose_stamped_pub.publish(pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = MirPoseSimple()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
