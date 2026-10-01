"""Publish QTM rigid-body poses in ROS 2 without upgrading the ROS 1 host."""

import copy
import json
import queue
import re
import subprocess
import threading
import time
from functools import partial
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.executors import ExternalShutdownException
from scipy.spatial.transform import Rotation

from match_mocap_ros2.planar_smoothing import PlanarMovingAverage
from match_mocap_ros2.map_transform import checked_map_transform


def pose_from_qtm(body, stamp, frame_id):
    """Convert the same millimetre and column-major data as the ROS 1 driver."""
    position = np.asarray(body['position_mm'], dtype=float)
    matrix = np.asarray(body['rotation_column_major'], dtype=float).reshape((3, 3), order='F')
    if not np.isfinite(position).all() or not np.isfinite(matrix).all():
        raise ValueError('QTM pose contains non-finite values')
    quaternion = Rotation.from_matrix(matrix).as_quat()
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position / 1000.0
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion
    return pose


ROBOT_NAMES = ('mur620a', 'mur620b', 'mur620c', 'mur620d')


class LocalizationHold:
    """Keep the last valid map pose while a robot is hidden from Qualisys."""

    MAX_POSE_AGE_SEC = 0.5

    def __init__(self):
        self.latest = {}
        self.frozen = {}

    def observe(self, robot, pose, received_at):
        self.latest[robot] = (received_at, copy.deepcopy(pose))

    def set_frozen(self, robot, enabled, now):
        if enabled and robot in self.frozen:
            return True, 'already frozen'
        if not enabled and robot not in self.frozen:
            return True, 'already following Qualisys'
        latest = self.latest.get(robot)
        if latest is None or now - latest[0] > self.MAX_POSE_AGE_SEC:
            return False, 'no fresh map pose; localization state unchanged'
        if enabled:
            self.frozen[robot] = copy.deepcopy(latest[1])
            return True, 'last valid map pose frozen'
        del self.frozen[robot]
        return True, 'following live Qualisys map pose'

    def frozen_pose(self, robot, stamp):
        pose = self.frozen.get(robot)
        if pose is None:
            return None
        result = copy.deepcopy(pose)
        result.header.stamp = stamp
        return result


def robot_tf_from_map_pose(mapped_pose, body_name):
    """Use the QTM base_link pose unchanged for the URDF's identity base joint."""
    if mapped_pose.header.frame_id != 'map':
        raise ValueError('Robot TF requires a map-frame Qualisys pose')
    transform = TransformStamped()
    transform.header = mapped_pose.header
    transform.child_frame_id = f'{body_name}/base_footprint'
    transform.transform.translation.x = mapped_pose.pose.position.x
    transform.transform.translation.y = mapped_pose.pose.position.y
    transform.transform.translation.z = mapped_pose.pose.position.z
    transform.transform.rotation = mapped_pose.pose.orientation
    return transform


class QualisysSshBridge(Node):
    def __init__(self):
        super().__init__('qualisys_ssh_bridge')
        self.ssh_host = str(self.declare_parameter('ssh_host', 'roscore').value)
        self.qtm_host = str(self.declare_parameter('qtm_host', 'QTM').value)
        self.qtm_port = int(self.declare_parameter('qtm_port', 22223).value)
        self.frequency = int(self.declare_parameter('frequency', 100).value)
        self.frame_id = str(self.declare_parameter('frame_id', 'mocap').value)
        self.publish_map_pose = bool(self.declare_parameter('publish_map_pose', True).value)
        self.publish_robot_tf = bool(self.declare_parameter('publish_robot_tf', False).value)
        self.smoothing_window_sec = float(self.declare_parameter('smoothing_window_sec', 0.2).value)
        self.smoothed_rate_hz = float(self.declare_parameter('smoothed_rate_hz', 10.0).value)
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', self.ssh_host):
            raise ValueError('ssh_host must be a plain SSH host name')
        if not re.fullmatch(r'[A-Za-z0-9_.:-]+', self.qtm_host):
            raise ValueError('qtm_host must be a plain host name or IP address')
        if not 1 <= self.qtm_port <= 65535 or not 1 <= self.frequency <= 200:
            raise ValueError('qtm_port or frequency outside valid range')
        if not 0 < self.smoothing_window_sec <= 5 or not 0 < self.smoothed_rate_hz <= self.frequency:
            raise ValueError('smoothing_window_sec or smoothed_rate_hz outside valid range')

        self.process = None
        self.messages = queue.Queue(maxsize=256)
        self._body_publishers = {}
        self._smoothed_publishers = {}
        self._map_publishers = {}
        self.map_transform = None
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.body_tf_broadcaster = TransformBroadcaster(self)
        self._smoothers = {}
        self._last_smoothed_at = {}
        self.localization_hold = LocalizationHold()
        freeze_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._freeze_status_publishers = {}
        for robot in ROBOT_NAMES:
            self.create_service(
                SetBool, f'/qualisys/{robot}/freeze_localization',
                partial(self.set_localization_frozen, robot)
            )
            self._freeze_status_publishers[robot] = self.create_publisher(
                Bool, f'/qualisys/{robot}/localization_frozen', freeze_qos
            )
        self.last_frame_at = 0.0
        self.next_connect_at = 0.0
        self.create_timer(0.005, self.pump)
        self.create_timer(0.05, self.publish_frozen_localizations)
        self.create_timer(1.0, self.publish_freeze_status)
        self.publish_freeze_status()
        self.start_remote()

    def set_localization_frozen(self, robot, request, response):
        response.success, response.message = self.localization_hold.set_frozen(
            robot, request.data, time.monotonic()
        )
        self.publish_freeze_status()
        if response.success:
            self.get_logger().info(f'{robot}: {response.message}')
        else:
            self.get_logger().warning(f'{robot}: {response.message}')
        return response

    def publish_freeze_status(self):
        for robot, publisher in self._freeze_status_publishers.items():
            message = Bool()
            message.data = robot in self.localization_hold.frozen
            publisher.publish(message)

    def publish_frozen_localizations(self):
        if not self.localization_hold.frozen:
            return
        stamp = self.get_clock().now().to_msg()
        for robot in tuple(self.localization_hold.frozen):
            mapped_pose = self.localization_hold.frozen_pose(robot, stamp)
            for suffix in ('pose', 'pose_smoothed'):
                topic = f'/qualisys_map/{robot}/{suffix}'
                if topic not in self._map_publishers:
                    self._map_publishers[topic] = self.create_publisher(PoseStamped, topic, 10)
                self._map_publishers[topic].publish(mapped_pose)
            if self.publish_robot_tf:
                self.body_tf_broadcaster.sendTransform(
                    robot_tf_from_map_pose(mapped_pose, robot)
                )

    def start_remote(self):
        source = Path(__file__).with_name('remote_qtm_stream.py').read_bytes()
        command = [
            'ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
            '-o', 'ServerAliveInterval=10', '-o', 'ServerAliveCountMax=3',
            self.ssh_host, 'python3', '-u', '-', self.qtm_host,
            str(self.qtm_port), str(self.frequency),
        ]
        try:
            self.process = subprocess.Popen(
                command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.process.stdin.write(source)
            self.process.stdin.close()
        except (OSError, BrokenPipeError) as exc:
            self.get_logger().error(f'Cannot start QTM SSH stream: {exc}')
            self.stop_remote()
            self.next_connect_at = time.monotonic() + 3.0
            return
        self.last_frame_at = time.monotonic()
        self.map_transform = None
        self._smoothers.clear()
        self._last_smoothed_at.clear()
        threading.Thread(target=self.read_stdout, args=(self.process,), daemon=True).start()
        threading.Thread(target=self.read_stderr, args=(self.process,), daemon=True).start()
        self.get_logger().info(f'Connecting to QTM {self.qtm_host}:{self.qtm_port} via SSH {self.ssh_host}')

    def read_stdout(self, process):
        for line in process.stdout:
            try:
                item = json.loads(line)
                try:
                    self.messages.put_nowait(item)
                except queue.Full:
                    self.messages.get_nowait()
                    self.messages.put_nowait(item)
            except (ValueError, queue.Empty):
                continue

    def read_stderr(self, process):
        for line in process.stderr:
            message = line.decode(errors='replace').strip()
            if message:
                if ' - INFO - ' in message:
                    self.get_logger().info(f'QTM SSH: {message}')
                else:
                    self.get_logger().warning(f'QTM SSH: {message}')

    def pump(self):
        now = time.monotonic()
        if self.process is None:
            if now >= self.next_connect_at:
                self.start_remote()
            return
        if self.process.poll() is not None or now - self.last_frame_at > 10.0:
            self.get_logger().warning('QTM stream stopped; reconnecting')
            self.stop_remote()
            self.next_connect_at = now + 3.0
            return

        for _ in range(20):
            try:
                item = self.messages.get_nowait()
            except queue.Empty:
                break
            if item.get('kind') == 'config':
                self.get_logger().info('QTM rigid bodies: ' + ', '.join(item.get('body_names', [])))
                self.configure_map_transform(item)
            elif item.get('kind') == 'frame':
                self.last_frame_at = now
                stamp = self.get_clock().now().to_msg()
                for body in item.get('bodies', []):
                    topic_name = re.sub(r'[^A-Za-z0-9_]', '_', body['name'])
                    if not topic_name:
                        continue
                    topic = f'/qualisys/{topic_name}/pose'
                    if topic not in self._body_publishers:
                        self._body_publishers[topic] = self.create_publisher(PoseStamped, topic, 10)
                        self.get_logger().info(f'Publishing {topic} in frame {self.frame_id}')
                    try:
                        pose = pose_from_qtm(body, stamp, self.frame_id)
                    except (ValueError, KeyError):
                        self.get_logger().warning(f'Ignoring invalid QTM pose for {topic_name}')
                        continue
                    self._body_publishers[topic].publish(pose)
                    if topic_name in ROBOT_NAMES:
                        self.publish_body_tf(topic_name, pose)
                        self.publish_map(topic_name, pose)
                        self.publish_smoothed(topic_name, pose, time.monotonic())

    def publish_smoothed(self, body_name, pose, received_at):
        smoother = self._smoothers.setdefault(
            body_name, PlanarMovingAverage(self.smoothing_window_sec)
        )
        smoothed = smoother.add(pose, received_at)
        if received_at - self._last_smoothed_at.get(body_name, 0.0) < 1.0 / self.smoothed_rate_hz:
            return
        topic = f'/qualisys/{body_name}/pose_smoothed'
        if topic not in self._smoothed_publishers:
            self._smoothed_publishers[topic] = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f'Publishing {topic} in frame {self.frame_id}')
        self._smoothed_publishers[topic].publish(smoothed)
        self.publish_map(body_name, smoothed, smoothed=True)
        self._last_smoothed_at[body_name] = received_at

    def configure_map_transform(self, config):
        self.clear_map_publishers()
        self.map_transform = None
        if not self.publish_map_pose:
            self.get_logger().info('Map pose publishing disabled')
            return
        if self.frame_id != 'mocap':
            self.get_logger().error('Map output disabled: source frame must be mocap')
            return
        if 'map_launch_error' in config:
            self.get_logger().error(f"Map output disabled: {config['map_launch_error']}")
            return
        try:
            self.map_transform = checked_map_transform(config['map_launch_xml'])
        except (KeyError, ValueError) as exc:
            self.get_logger().error(f'Map output disabled: invalid ROS 1 transform: {exc}')
            return
        transform = self.map_transform
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'map'
        static_tf.child_frame_id = 'mocap'
        static_tf.transform.translation.x = transform.x
        static_tf.transform.translation.y = transform.y
        static_tf.transform.translation.z = transform.z
        static_tf.transform.rotation.z = np.sin(transform.yaw / 2.0)
        static_tf.transform.rotation.w = np.cos(transform.yaw / 2.0)
        self.static_tf_broadcaster.sendTransform(static_tf)
        self.get_logger().info(
            f'Using roscore map -> mocap: x={transform.x:.4f} m, '
            f'y={transform.y:.4f} m, z={transform.z:.4f} m, yaw={transform.yaw:.6f} rad'
        )

    def publish_body_tf(self, body_name, pose):
        if self.map_transform is None:
            return
        body_tf = TransformStamped()
        body_tf.header = pose.header
        body_tf.child_frame_id = f'qualisys/{body_name}'
        body_tf.transform.translation.x = pose.pose.position.x
        body_tf.transform.translation.y = pose.pose.position.y
        body_tf.transform.translation.z = pose.pose.position.z
        body_tf.transform.rotation = pose.pose.orientation
        self.body_tf_broadcaster.sendTransform(body_tf)

    def publish_map(self, body_name, pose, smoothed=False):
        if self.map_transform is None:
            return
        suffix = 'pose_smoothed' if smoothed else 'pose'
        topic = f'/qualisys_map/{body_name}/{suffix}'
        if topic not in self._map_publishers:
            self._map_publishers[topic] = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f'Publishing {topic} in frame map')
        mapped_pose = self.map_transform.apply(pose)
        if not smoothed and body_name in ROBOT_NAMES:
            self.localization_hold.observe(body_name, mapped_pose, time.monotonic())
        if body_name in self.localization_hold.frozen:
            return
        self._map_publishers[topic].publish(mapped_pose)
        if not smoothed and self.publish_robot_tf and body_name in ROBOT_NAMES:
            self.body_tf_broadcaster.sendTransform(
                robot_tf_from_map_pose(mapped_pose, body_name)
            )

    def clear_map_publishers(self):
        for publisher in self._map_publishers.values():
            self.destroy_publisher(publisher)
        self._map_publishers.clear()

    def stop_remote(self):
        self.map_transform = None
        self.clear_map_publishers()
        process = self.process
        self.process = None
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)

    def destroy_node(self):
        self.stop_remote()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QualisysSshBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
