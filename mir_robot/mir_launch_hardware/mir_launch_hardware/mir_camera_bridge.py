#!/usr/bin/env python3
"""Bridge the MiR floor cameras from ROS 1 rosbridge to namespaced ROS 2 topics."""

import base64
import threading
import time
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from mir_driver.rosbridge import RosbridgeSetup


CAMERA_STREAMS = {
    'color/image_raw': Image,
    'color/camera_info': CameraInfo,
    'depth/image_rect_raw': Image,
    'depth/camera_info': CameraInfo,
}


def convert_header(source, namespace):
    header = Header()
    stamp = source.get('stamp', {})
    header.stamp.sec = int(stamp.get('secs', 0))
    header.stamp.nanosec = int(stamp.get('nsecs', 0))
    frame_id = str(source.get('frame_id', '')).strip('/')
    namespace = namespace.strip('/')
    if namespace and frame_id and not frame_id.startswith(namespace + '/'):
        frame_id = namespace + '/' + frame_id
    header.frame_id = frame_id
    return header


def convert_image(source, namespace):
    image = Image()
    image.header = convert_header(source.get('header', {}), namespace)
    image.height = int(source['height'])
    image.width = int(source['width'])
    image.encoding = str(source['encoding'])
    image.is_bigendian = int(source.get('is_bigendian', 0))
    image.step = int(source['step'])
    payload = source['data']
    if isinstance(payload, str):
        payload = base64.b64decode(payload, validate=True)
    else:
        payload = bytes(payload)
    if len(payload) != image.height * image.step:
        raise ValueError(
            f'image payload has {len(payload)} bytes; expected {image.height * image.step}'
        )
    image.data = payload
    return image


def convert_camera_info(source, namespace):
    info = CameraInfo()
    info.header = convert_header(source.get('header', {}), namespace)
    info.height = int(source['height'])
    info.width = int(source['width'])
    info.distortion_model = str(source.get('distortion_model', ''))
    info.d = [float(value) for value in source.get('D', [])]
    info.k = [float(value) for value in source['K']]
    info.r = [float(value) for value in source['R']]
    info.p = [float(value) for value in source['P']]
    info.binning_x = int(source.get('binning_x', 0))
    info.binning_y = int(source.get('binning_y', 0))
    roi = source.get('roi', {})
    info.roi.x_offset = int(roi.get('x_offset', 0))
    info.roi.y_offset = int(roi.get('y_offset', 0))
    info.roi.height = int(roi.get('height', 0))
    info.roi.width = int(roi.get('width', 0))
    info.roi.do_rectify = bool(roi.get('do_rectify', False))
    return info


class MiRCameraBridge(Node):
    def __init__(self):
        super().__init__('mir_camera_bridge')
        self.hostname = str(self.declare_parameter('mir_hostname', '192.168.12.20').value)
        self.port = int(self.declare_parameter('mir_port', 9090).value)
        sides = str(self.declare_parameter('camera_sides', 'left right').value).replace(',', ' ').split()
        self.sides = list(dict.fromkeys(sides))
        if not self.sides or any(side not in ('left', 'right') for side in self.sides):
            raise ValueError('camera_sides must contain left, right, or both')
        rate_hz = float(self.declare_parameter('max_rate_hz', 2.0).value)
        if rate_hz <= 0:
            raise ValueError('max_rate_hz must be positive')
        self.throttle_ms = max(1, round(1000.0 / rate_hz))
        self.camera_namespace = self.get_namespace().strip('/')
        self.camera_publishers = {}
        for side in self.sides:
            for suffix, msg_type in CAMERA_STREAMS.items():
                topic = f'/camera_floor_{side}/driver/{suffix}'
                self.camera_publishers[topic] = self.create_publisher(
                    msg_type, topic.lstrip('/'), qos_profile_sensor_data
                )

        self.robot = None
        self.connected_since = 0.0
        self.next_connect_at = 0.0
        self.subscribed = False
        self.pending = {}
        self.first_frames = set()
        self.lock = threading.Lock()
        self.create_timer(1.0, self.ensure_connection)
        self.create_timer(0.05, self.publish_pending)
        self.ensure_connection()

    def ensure_connection(self):
        now = time.monotonic()
        if self.robot is not None and self.robot.is_connected():
            if not self.subscribed:
                for topic in self.camera_publishers:
                    self.robot.subscribe(
                        topic, partial(self.queue_message, topic), throttle_rate=self.throttle_ms
                    )
                self.subscribed = True
                self.get_logger().info(
                    f'Streaming MiR cameras {self.sides} from {self.hostname}:{self.port} '
                    f'at most {1000.0 / self.throttle_ms:.1f} Hz per topic'
                )
            return
        if self.robot is not None:
            if not (self.robot.is_errored() or self.subscribed or now - self.connected_since > 8.0):
                return
            self.robot.connection.ws.close()
            self.robot = None
            self.subscribed = False
            self.next_connect_at = now + 2.0
            self.get_logger().warning('MiR camera rosbridge disconnected; reconnecting')
        if now >= self.next_connect_at:
            self.robot = RosbridgeSetup(self.hostname, self.port)
            self.connected_since = now

    def queue_message(self, topic, payload):
        if isinstance(payload, dict):
            with self.lock:
                self.pending[topic] = payload

    def publish_pending(self):
        with self.lock:
            pending = self.pending
            self.pending = {}
        for topic, payload in pending.items():
            publisher = self.camera_publishers[topic]
            if publisher.get_subscription_count() == 0:
                continue
            try:
                if CAMERA_STREAMS[topic.rsplit('/driver/', 1)[1]] is Image:
                    message = convert_image(payload, self.camera_namespace)
                else:
                    message = convert_camera_info(payload, self.camera_namespace)
                publisher.publish(message)
                if topic not in self.first_frames:
                    self.first_frames.add(topic)
                    self.get_logger().info(f'First MiR camera frame published: {topic}')
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().warning(
                    f'Invalid MiR camera message on {topic}: {exc}',
                    throttle_duration_sec=5.0,
                )

    def destroy_node(self):
        if self.robot is not None:
            self.robot.connection.ws.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MiRCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
