"""Gate sensor input and maintain a private mapping TF tree and retained map."""
import copy
import json
import math
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from .settings import DEFAULTS, local_transform, map_frame, validate

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def corrected(message, seconds):
    result = copy.deepcopy(message)
    stamp = result.header.stamp
    value = stamp.sec * 1000000000 + stamp.nanosec + round(seconds * 1000000000)
    stamp.sec, stamp.nanosec = divmod(value, 1000000000)
    return result


def valid_pose(msg):
    p, q = msg.pose.position, msg.pose.orientation
    return (msg.header.frame_id == 'mocap' and
            all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) and
            abs(sum(v*v for v in (q.x, q.y, q.z, q.w)) - 1) <= 0.01)


class MappingSession(Node):
    def __init__(self, **kwargs):
        super().__init__('mapping_session', **kwargs)
        self.robot = self.declare_parameter('robot', 'mur620d').value
        self.mode = self.declare_parameter('mode', 'mocap').value
        self.session_id = self.declare_parameter('session_id', uuid.uuid4().hex).value
        self.settings = validate(self.robot, self.mode, {
            name: self.declare_parameter(name, default).value for name, default in DEFAULTS.items()})
        if self.count_publishers('status'):
            raise RuntimeError('Another mapping session already owns this namespace')
        self.recording = True
        self.received = {}
        self.static_edges = {}
        self.map = None
        self.mapper_status = {}
        self.started = time.monotonic()
        self.tf_pub = self.create_publisher(TFMessage, 'tf', 100)
        self.static_pub = self.create_publisher(TFMessage, 'tf_static', LATCHED)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 100)
        self.pose_display_pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', LATCHED)
        self.status_pub = self.create_publisher(String, 'status', LATCHED)
        self.record_pub = self.create_publisher(Bool, 'recording', LATCHED)
        self.scan_pubs = {name: self.create_publisher(LaserScan, name, qos_profile_sensor_data)
                          for name in ('front_scan', 'rear_scan', 'scan')}
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(TFMessage, '/tf_static', self.on_static, LATCHED)
        self.create_subscription(PoseStamped, f'/qualisys/{self.robot}/pose', self.on_pose, 100)
        self.create_subscription(Odometry, f'/{self.robot}/odom', self.on_odom, qos_profile_sensor_data)
        for source, target, offset in (
                ('f_scan', 'front_scan', 'front_time_offset'),
                ('b_scan', 'rear_scan', 'rear_time_offset'), ('scan', 'scan', None)):
            self.create_subscription(LaserScan, f'/{self.robot}/{source}',
                lambda msg, t=target, o=offset: self.on_scan(msg, t, o), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, 'map_raw', self.on_map, LATCHED)
        self.create_subscription(String, 'mapper_status', self.on_mapper_status, 10)
        self.create_service(SetBool, 'set_recording', self.set_recording)
        self.create_timer(0.2, self.publish_status)
        self.record_pub.publish(Bool(data=True))

    def on_tf(self, msg):
        transforms = [t for t in msg.transforms if local_transform(t, self.robot)]
        if transforms:
            self.tf_pub.publish(TFMessage(transforms=transforms))

    def on_static(self, msg):
        for t in msg.transforms:
            if local_transform(t, self.robot):
                self.static_edges[t.child_frame_id] = t
        if self.static_edges:
            self.static_pub.publish(TFMessage(transforms=list(self.static_edges.values())))

    def fresh(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return abs(self.get_clock().now().nanoseconds * 1e-9 - stamp) <= self.settings['pose_timeout']

    def on_pose(self, msg):
        if not valid_pose(msg):
            return
        msg = corrected(msg, self.settings['mocap_time_offset'])
        if not self.fresh(msg):
            return
        self.received['mocap'] = time.monotonic()
        if self.mode != 'mocap':
            return
        self.pose_pub.publish(msg)
        self.pose_display_pub.publish(msg)
        t = TransformStamped(header=msg.header, child_frame_id=f'{self.robot}/base_footprint')
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[t]))

    def on_odom(self, msg):
        if not self.fresh(msg):
            return
        if self.mode != 'slam':
            self.received['odom'] = time.monotonic()
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01:
            return
        # MuR odometry describes the base. Reject unexpected child frames instead of guessing offsets.
        if msg.child_frame_id.strip('/') not in ('base_link', 'base_footprint',
                f'{self.robot}/base_link', f'{self.robot}/base_footprint'):
            return
        self.received['odom'] = time.monotonic()
        t = TransformStamped(header=copy.deepcopy(msg.header), child_frame_id=f'{self.robot}/base_footprint')
        t.header.frame_id = f'{self.robot}/mapping/odom'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = q
        self.tf_pub.publish(TFMessage(transforms=[t]))

    def on_scan(self, msg, target, offset):
        if offset:
            msg = corrected(msg, self.settings[offset])
        if not self.fresh(msg):
            return
        self.received[target] = time.monotonic()
        if not self.recording or (self.mode == 'slam' and target != 'scan') or (
                self.mode == 'mocap' and target == 'scan'):
            return
        if self.mode == 'slam':
            msg = copy.deepcopy(msg)
            low = max(msg.range_min, self.settings['min_range'])
            high = min(msg.range_max, self.settings['max_range'])
            msg.ranges = [float('nan') if math.isnan(r) or r < low or (math.isfinite(r) and r > msg.range_max)
                          else float('inf') if r >= high else r for r in msg.ranges]
            msg.range_min, msg.range_max = low, high
        self.scan_pubs[target].publish(msg)

    def on_map(self, msg):
        if msg.header.frame_id != map_frame(self.robot, self.mode):
            return
        if msg.info.width * msg.info.height != len(msg.data) or not msg.data:
            return
        self.map = msg
        self.map_pub.publish(msg)

    def on_mapper_status(self, msg):
        try:
            self.mapper_status = json.loads(msg.data)
        except (ValueError, TypeError):
            pass

    def set_recording(self, request, response):
        if request.data and not self.recording:
            response.success = False
            response.message = 'Stopped sessions cannot resume; start a new session'
            return response
        self.recording = bool(request.data)
        self.record_pub.publish(Bool(data=self.recording))
        response.success = True
        response.message = 'Recording' if self.recording else 'Input stopped; last map retained'
        self.publish_status()
        return response

    def publish_status(self):
        now = time.monotonic()
        live = {k: now - self.received.get(k, -math.inf) < self.settings['pose_timeout']
                for k in ('mocap', 'odom', 'front_scan', 'rear_scan', 'scan')}
        source = live['mocap' if self.mode == 'mocap' else 'odom']
        scans = live['front_scan'] or live['rear_scan'] if self.mode == 'mocap' else live['scan']
        state = 'recording' if source and scans else (
            'waiting_for_mocap' if self.mode == 'mocap' and not source else 'waiting_for_input')
        if self.mode == 'slam' and state == 'recording' and self.map is None:
            state = 'waiting_for_map'
        if self.mode == 'mocap' and state == 'recording':
            state = self.mapper_status.get('reason', 'waiting_for_mapper')
        map_stamp = (self.map.header.stamp.sec * 1000000000 + self.map.header.stamp.nanosec) if self.map else 0
        status = dict(robot=self.robot, mode=self.mode, session_id=self.session_id,
                      map_stamp=map_stamp, recording=self.recording,
                      state=state if self.recording else 'stopped', live=live,
                      has_map=self.map is not None, **{k: self.mapper_status.get(k, 0)
                      for k in ('integrated', 'discarded')})
        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    rclpy.init()
    node = None
    try:
        node = MappingSession()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
