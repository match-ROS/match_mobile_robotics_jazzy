"""Independent wheel-odometry TF, one-time initialization and Mocap comparison.

Nothing is published to hardware TF, initialpose or velocity topics. Mocap is
used once for initialization and subsequently only as the measurement reference.
"""
from collections import deque
import copy
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import time
import uuid

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from tf2_msgs.msg import TFMessage

from .comparison import distribution, errors, interpolate, yaw
from .session import LATCHED, valid_pose
from .settings import ROBOTS, local_transform


def seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec*1e-9


class LocalizationSession(Node):
    def __init__(self, **kwargs):
        super().__init__('localization_session', **kwargs)
        self.robot = self.declare_parameter('robot', 'mur620c').value
        if self.robot not in ROBOTS:
            raise ValueError('Unknown MuR')
        self.map_yaml = self.declare_parameter('map_yaml', '').value
        self.stationary_updates = self.declare_parameter('stationary_updates', False).value
        source = self.declare_parameter('sensor_namespace', '').value.rstrip('/')
        self.sensor_namespace = source or '/'+self.robot
        directory = Path(self.declare_parameter('output_directory', '/tmp/mur_localization').value)
        directory.mkdir(parents=True, exist_ok=True)
        self.output = directory / (self.robot+'_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        self.csv_file = self.output.with_suffix('.csv').open('w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['stamp', 'amcl_x_m', 'amcl_y_m', 'amcl_yaw_rad',
                              'mocap_x_m', 'mocap_y_m', 'mocap_yaw_rad',
                              'dx_m', 'dy_m', 'position_error_m', 'yaw_error_deg'])
        self.reference = deque(maxlen=12000)
        self.pending = deque()
        self.position_errors, self.yaw_errors = [], []
        self.reference_positions = []
        self.latest_pose = None
        self.initialized_at = None
        self.received = {}
        self.last_amcl_pose = None
        self.stationary_update_changes = []
        self.rejected = dict(warmup=0, unmatched=0, invalid=0)
        self.static_edges = {}
        self.state_future = self.update_future = None
        self.last_update = 0.0
        self.seed_active = False
        self.tf_pub = self.create_publisher(TFMessage, 'tf', 100)
        self.static_pub = self.create_publisher(TFMessage, 'tf_static', LATCHED)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, 'mocap_pose', 10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.status_pub = self.create_publisher(String, 'comparison', LATCHED)
        self.state_client = self.create_client(GetState, 'amcl/get_state')
        self.update_client = self.create_client(Empty, 'request_nomotion_update')
        self.create_subscription(TFMessage, (source+'/tf') if source else '/tf', self.on_tf, 100)
        self.create_subscription(TFMessage, (source+'/tf_static') if source else '/tf_static', self.on_static, LATCHED)
        self.create_subscription(Odometry, self.sensor_namespace+'/odom_enc', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.sensor_namespace+'/f_scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f'/qualisys/{self.robot}/pose', self.on_mocap, 100)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, LATCHED)
        self.create_subscription(TFMessage, 'tf', self.observe_amcl_tf, 100)
        self.create_service(SetBool, 'set_stationary_updates', self.set_stationary_updates)
        self.create_timer(0.2, self.tick)
        self.create_timer(2.0, self.report)
        self.get_logger().info('Vergleichsdateien: '+str(self.output)+'.{csv,json}')

    def fresh(self, message):
        return abs(self.get_clock().now().nanoseconds*1e-9 - seconds(message)) <= 0.5

    def on_tf(self, message):
        edges = [t for t in message.transforms if local_transform(t, self.robot)]
        if edges:
            self.tf_pub.publish(TFMessage(transforms=edges))

    def on_static(self, message):
        for t in message.transforms:
            if local_transform(t, self.robot):
                self.static_edges[t.child_frame_id] = t
        if self.static_edges:
            self.static_pub.publish(TFMessage(transforms=list(self.static_edges.values())))

    def on_odom(self, message):
        # Explicit encoder topic only. Never use a Mocap/fused map pose as odometry.
        if not self.fresh(message):
            return
        if message.child_frame_id.strip('/') != self.robot+'/base_footprint':
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        if abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1) > 0.01:
            return
        t = TransformStamped(header=copy.deepcopy(message.header), child_frame_id=self.robot+'/base_footprint')
        t.header.frame_id = self.robot+'/localization/odom'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = q
        self.tf_pub.publish(TFMessage(transforms=[t]))
        self.received['encoder_odom'] = time.monotonic()

    def on_scan(self, message):
        if self.fresh(message) and message.header.frame_id.startswith(self.robot+'/'):
            self.scan_pub.publish(message)
            self.received['front_scan'] = time.monotonic()

    def on_mocap(self, message):
        if not valid_pose(message) or not self.fresh(message):
            return
        stamp = seconds(message)
        if self.reference and stamp <= self.reference[-1][0]:
            return
        p = message.pose.position
        self.reference.append((stamp, p.x, p.y, yaw(message.pose.orientation)))
        self.latest_pose = message
        self.pose_pub.publish(message)
        self.received['mocap'] = time.monotonic()

    def on_amcl(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if (message.header.frame_id != 'mocap' or not self.fresh(message) or
                not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)) or
                abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1) > 0.01):
            self.rejected['invalid'] += 1
            return
        self.received['amcl'] = time.monotonic()
        self.last_amcl_pose = message
        stamp = seconds(message)
        if self.initialized_at is None or stamp < self.initialized_at + 5.0:
            self.rejected['warmup'] += 1
            return
        self.pending.append((time.monotonic(), stamp, (p.x, p.y, yaw(q))))

    def observe_amcl_tf(self, message):
        for t in message.transforms:
            if t.header.frame_id == 'mocap' and t.child_frame_id == self.robot+'/localization/odom':
                # AMCL republishes its transform on scans even while the robot
                # rests; amcl_pose itself is only emitted on filter updates.
                age = self.get_clock().now().nanoseconds*1e-9 - seconds(t)
                if -2.0 <= age <= 0.5:
                    self.received['amcl_tf'] = time.monotonic()

    def set_stationary_updates(self, request, response):
        self.stationary_updates = request.data
        self.stationary_update_changes.append(dict(
            stamp=self.get_clock().now().nanoseconds*1e-9, enabled=request.data))
        response.success = True
        response.message = ('Standmessung aktiviert (ca. 1 Hz).' if request.data else
                            'Standmessung deaktiviert; AMCL aktualisiert bei Bewegung.')
        self.report()
        return response

    def tick(self):
        # Wait until lifecycle activation before publishing exactly one seed.
        if self.initialized_at is None:
            if self.state_future is not None and self.state_future.done():
                try:
                    self.seed_active = self.state_future.result().current_state.id == 3
                except Exception:
                    self.seed_active = False
                self.state_future = None
            if self.seed_active and self.latest_pose is not None and self.fresh(self.latest_pose):
                if self.initial_pub.get_subscription_count() and all(
                        time.monotonic()-self.received.get(key, -100) < 0.5
                        for key in ('encoder_odom', 'front_scan')):
                    seed = PoseWithCovarianceStamped(header=copy.deepcopy(self.latest_pose.header))
                    seed.pose.pose = copy.deepcopy(self.latest_pose.pose)
                    seed.pose.covariance[0] = seed.pose.covariance[7] = 0.25**2
                    seed.pose.covariance[35] = 0.15**2
                    self.initial_pub.publish(seed)
                    self.initialized_at = self.get_clock().now().nanoseconds*1e-9
                    self.get_logger().info('AMCL einmalig aus Mocap initialisiert; Messung nach 5 s Einlaufzeit.')
            elif self.state_future is None and self.state_client.service_is_ready():
                self.state_future = self.state_client.call_async(GetState.Request())
        if self.stationary_updates and self.initialized_at is not None and time.monotonic()-self.last_update >= 1:
            if self.update_client.service_is_ready() and (self.update_future is None or self.update_future.done()):
                self.update_future = self.update_client.call_async(Empty.Request())
                self.last_update = time.monotonic()
        reference = list(self.reference)
        while self.pending:
            arrived, stamp, estimate = self.pending[0]
            truth = interpolate(reference, stamp)
            if truth is None and time.monotonic()-arrived < 0.5:
                break
            self.pending.popleft()
            if truth is None:
                self.rejected['unmatched'] += 1
                continue
            result = errors(estimate, truth)
            self.writer.writerow((stamp, *estimate, *truth, *result))
            self.position_errors.append(result[2])
            self.yaw_errors.append(abs(result[3]))
            self.reference_positions.append(truth[:2])

    def report(self):
        now = time.monotonic()
        live = {key: now-self.received.get(key, -100) < 1.0
                for key in ('encoder_odom', 'front_scan', 'mocap', 'amcl')}
        live['amcl_updates'] = live['amcl']
        live['amcl'] = self.last_amcl_pose is not None and now-self.received.get('amcl_tf', -100) < 1.0
        positions = self.reference_positions
        span = (math.hypot(max(p[0] for p in positions)-min(p[0] for p in positions),
                           max(p[1] for p in positions)-min(p[1] for p in positions)) if positions else 0.0)
        summary = dict(robot=self.robot, map_yaml=self.map_yaml, frame='mocap',
            odom_topic=self.sensor_namespace+'/odom_enc', scan_topic=self.sensor_namespace+'/f_scan',
            initialized_from_mocap_once=self.initialized_at is not None,
            warmup_seconds=5.0, reference_max_gap_seconds=0.1,
            stationary_updates=self.stationary_updates, samples=len(self.position_errors),
            stationary_update_changes=list(self.stationary_update_changes),
            amcl_pose_available=self.last_amcl_pose is not None,
            amcl_pose_age_seconds=(self.get_clock().now().nanoseconds*1e-9-seconds(self.last_amcl_pose)
                                   if self.last_amcl_pose is not None else None),
            position_m=distribution(self.position_errors), yaw_deg=distribution(self.yaw_errors),
            reference_span_m=span, rejected=dict(self.rejected), live=live,
            output=str(self.output),
            limitation='Map created using Mocap; one-time Mocap seed. Stationary samples are correlated; '
                       'a stationary run does not measure trajectory accuracy or global relocalization.')
        self.csv_file.flush()
        temporary = self.output.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(summary, indent=2), encoding='utf-8')
        temporary.replace(self.output.with_suffix('.json'))
        if self.context.ok():
            self.status_pub.publish(String(data=json.dumps(summary)))
        return summary

    def destroy_node(self):
        try:
            self.report()
        finally:
            self.csv_file.close()
        return super().destroy_node()


def main():
    # Keep the ROS context valid until callbacks have stopped and reports flush.
    # rclpy's asynchronous shutdown otherwise races active publishers on SIGINT.
    def stop(_signal, _frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LocalizationSession()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
