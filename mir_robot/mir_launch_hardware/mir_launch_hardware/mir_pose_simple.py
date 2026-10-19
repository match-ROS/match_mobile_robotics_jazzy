#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage
from tf2_geometry_msgs import do_transform_pose
from rclpy.time import Time


class AutomaticLocalization:
    """Select fresh map-frame poses in a fixed, caller-provided source order."""

    def __init__(self, timeout, sources=('mocap', 'mir')):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('localization_timeout must be positive and finite')
        self.timeout = timeout
        self.poses = {}
        self.sources = sources

    def observe(self, source, pose, received_at, ros_now):
        if pose.header.frame_id != 'map':
            return False
        p, q = pose.pose.position, pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return False
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.01:
            return False
        stamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
        if abs(ros_now - stamp) > self.timeout:
            return False
        self.poses[source] = (received_at, pose)
        return True

    def select(self, now):
        for source in self.sources:
            latest = self.poses.get(source)
            if latest is not None and 0 <= now - latest[0] <= self.timeout:
                return source, latest[1]
        return 'waiting', None


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
        self.publish_robot_tf = self.declare_parameter('publish_robot_tf', False).value
        self.tf_prefix = self.declare_parameter('tf_prefix', self.get_namespace()).value.strip('/')

        self.pose_pub = self.create_publisher(Pose, 'mir_pose_simple', 1)
        self.pose_stamped_pub = self.create_publisher(PoseStamped, 'mir_pose_stamped_simple', 1)
        self._input_subscriptions = []

        if self.localization_type == 'auto' or (self.publish_robot_tf and
                self.localization_type in ('mir', 'mocap', 'amcl')):
            if self.output_frame != 'map':
                raise ValueError('Automatic localization requires output_frame=map')
            self.localization = AutomaticLocalization(
                float(self.declare_parameter('localization_timeout', 1.0).value),
                sources=('mocap', 'mir') if self.localization_type == 'auto' else (self.localization_type,),
            )
            self.localization_source = None
            if self.publish_robot_tf:
                self.tf_broadcaster = TransformBroadcaster(self)
                # Presence of this publisher also tells the standalone Mocap
                # bridge to leave this robot's base TF to this selector.
                self.source_pub = self.create_publisher(
                    String, 'localization_source',
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                )
            if self.localization_type in ('auto', 'mir'):
                self._input_subscriptions.append(self.create_subscription(
                    Pose, self.robot_pose_topic, self.auto_mir_callback, 10))
            if self.localization_type in ('auto', 'mocap'):
                self._input_subscriptions.append(self.create_subscription(
                    PoseStamped, self.mocap_topic, self.auto_mocap_callback, 10))
            if self.localization_type == 'amcl':
                # Keep private AMCL TF separate from hardware TF: both trees
                # contain base_footprint, with intentionally different parents.
                self.amcl_tf = Buffer()
                self.alignment_tf = Buffer()
                ns = '/'+self.tf_prefix+'/localization'
                self._input_subscriptions.extend([
                    self.create_subscription(TFMessage, ns+'/tf', self.amcl_tf_callback, 100),
                    self.create_subscription(TFMessage, '/tf_static', self.alignment_callback,
                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
                ])
            self.create_timer(0.05, self.publish_auto_pose)
        elif self.publish_robot_tf:
            raise ValueError('publish_robot_tf requires localization_type=auto/mir/mocap/amcl')
        elif self.localization_type == 'amcl':
            self._input_subscriptions.append(self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose_topic, self.amcl_pose_callback, 10))
        elif self.localization_type == 'ground_truth':
            self._input_subscriptions.append(self.create_subscription(Odometry, self.ground_truth_topic, self.odom_callback, 10))
        elif self.localization_type == 'odom':
            self._input_subscriptions.append(self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10))
        elif self.localization_type == 'mocap':
            self._input_subscriptions.append(self.create_subscription(PoseStamped, self.mocap_topic, self.mocap_callback, 10))
        elif self.localization_type == 'robot_pose':
            self._input_subscriptions.append(self.create_subscription(Pose, self.robot_pose_topic, self.robot_pose_callback, 10))
        else:
            self.get_logger().warning('Unknown localization_type: {}'.format(self.localization_type))

    def auto_mir_callback(self, msg):
        stamped = PoseStamped()
        stamped.header.frame_id = 'map'
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = msg
        self.observe_auto_pose('mir', stamped)

    def auto_mocap_callback(self, msg):
        self.observe_auto_pose('mocap', msg)

    def observe_auto_pose(self, source, msg):
        self.localization.observe(
            source, msg, time.monotonic(), self.get_clock().now().nanoseconds * 1e-9
        )

    def publish_auto_pose(self):
        if self.localization_type == 'amcl':
            self.observe_amcl_tracking()
        source, pose = self.localization.select(time.monotonic())
        if source != self.localization_source:
            self.localization_source = source
            self.get_logger().info(f'Base localization source: {source}')
            if self.publish_robot_tf:
                self.source_pub.publish(String(data=source))
        if pose is None:
            return
        # Both sources own the same map -> base edge, so changing source never
        # changes the TF tree's parent or competes with the URDF's base joint.
        stamped = PoseStamped()
        stamped.header.frame_id = 'map'
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = pose.pose
        self._publish(stamped.pose, stamped.header)
        if self.publish_robot_tf:
            transform = TransformStamped()
            transform.header = stamped.header
            transform.child_frame_id = '/'.join(filter(None, (self.tf_prefix, 'base_footprint')))
            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.translation.z = pose.pose.position.z
            transform.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def amcl_tf_callback(self, msg):
        allowed = {('mocap', self.tf_prefix+'/localization/odom'),
                   (self.tf_prefix+'/localization/odom', self.tf_prefix+'/base_footprint')}
        for transform in msg.transforms:
            if (transform.header.frame_id, transform.child_frame_id) in allowed:
                self.amcl_tf.set_transform(transform, 'amcl_experiment')

    def alignment_callback(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id, transform.child_frame_id) == ('map', 'mocap'):
                self.alignment_tf.set_transform_static(transform, 'mocap_calibration')

    def observe_amcl_tracking(self):
        try:
            tracking = self.amcl_tf.lookup_transform('mocap', self.tf_prefix+'/base_footprint', Time())
            alignment = self.alignment_tf.lookup_transform('map', 'mocap', Time())
        except Exception:
            return  # Missing calibration or AMCL: stay waiting, never use MiR as fallback.
        pose = PoseStamped()
        pose.header = tracking.header
        p = tracking.transform.translation
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p.x, p.y, p.z
        pose.pose.orientation = tracking.transform.rotation
        pose.pose = do_transform_pose(pose.pose, alignment)
        pose.header.frame_id = 'map'
        self.observe_auto_pose('amcl', pose)

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
