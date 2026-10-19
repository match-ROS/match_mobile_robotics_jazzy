"""Nav2 input/TF adapter and supervised NavigateToPose command ownership.

The stack has a private TF tree. Hardware commands are forwarded only for a
goal accepted by this action proxy, while all inputs and lifecycle nodes are live.
"""
from collections import deque
from pathlib import Path
import copy
import json
import math
import signal
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from action_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped

from .comparison import interpolate, yaw
from .navigation_config import DEFAULTS, validate
from .session import LATCHED, valid_pose
from .settings import ROBOTS, local_transform


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def planar_transform(parent, child, pose, when):
    t = TransformStamped(child_frame_id=child)
    t.header.frame_id, t.header.stamp = parent, when
    t.transform.translation.x, t.transform.translation.y = float(pose[0]), float(pose[1])
    t.transform.rotation.z, t.transform.rotation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
    return t


def map_to_odom(localized, odometry):
    angle = localized[2] - odometry[2]
    c, s = math.cos(angle), math.sin(angle)
    return (localized[0]-c*odometry[0]+s*odometry[1],
            localized[1]-s*odometry[0]-c*odometry[1], angle)


class NavigationSession(Node):
    def __init__(self, **kwargs):
        super().__init__('navigation_session', **kwargs)
        self.robot = self.declare_parameter('robot', 'mur620c').value
        self.source = self.declare_parameter('localization_source', 'mocap').value
        if self.robot not in ROBOTS or self.source not in ('mocap', 'amcl'):
            raise ValueError('Invalid robot or localization source')
        self.settings = validate({k: self.declare_parameter(k, v).value for k, v in DEFAULTS.items()})
        self.map_yaml = self.declare_parameter('map_yaml','').value
        self.amcl_map_matches = False
        sensor_ns = self.declare_parameter('sensor_namespace', '').value.rstrip('/')
        self.sensors = sensor_ns or '/'+self.robot
        self.output_topic = self.declare_parameter('output_topic', '/'+self.robot+'/cmd_vel_stamped').value
        self.base, self.odom_frame = self.robot+'/base_footprint', self.robot+'/navigation/odom'
        if self.count_publishers('status'):
            raise RuntimeError('Another navigation session owns this robot')
        self.private = Buffer(node=self)
        self.calibration = Buffer()
        self.amcl = Buffer()
        self.static_edges = {}
        self.mocap = deque(maxlen=2000)
        self.received, self.scan_frames, self.states, self.state_futures = {}, {}, {}, {}
        self.managed = ['map_server', 'planner_server', 'controller_server', 'behavior_server',
                        'bt_navigator', 'velocity_smoother', 'collision_monitor']
        self.state_clients = {name: self.create_client(GetState, name+'/get_state') for name in self.managed}
        self.encoder = None
        self.localized = None
        self.map = None
        self.native_busy = False
        self.busy = self.motion_enabled = False
        self.shutting_down = False
        self.backend_handle = self.front_handle = None
        self.cancel_requested = False
        self.failure = ''
        self.latched_fault = ''
        self.status_text = 'Bereit'
        self.distance_remaining = None
        self.command = None
        self.command_at = 0.
        self.hold_zero_until = self.reject_until = 0.
        self.last_velocity = (0., 0.)
        self.waiters = []
        self.tf_pub = self.create_publisher(TFMessage, 'tf', 100)
        self.static_pub = self.create_publisher(TFMessage, 'tf_static', LATCHED)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.status_pub = self.create_publisher(String, 'status', LATCHED)
        self.velocity_pub = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.scan_pubs = {key: self.create_publisher(LaserScan, key+'_scan', qos_profile_sensor_data)
                          for key in ('front','rear')}
        raw_tf = sensor_ns+'/tf' if sensor_ns else '/tf'
        raw_static = sensor_ns+'/tf_static' if sensor_ns else '/tf_static'
        self.create_subscription(TFMessage, raw_tf, lambda m: self.on_tf(m, False), 100)
        self.create_subscription(TFMessage, raw_static, lambda m: self.on_tf(m, True), LATCHED)
        for suffix, static in (('tf', False), ('tf_static', True)):
            self.create_subscription(TFMessage, f'/{self.robot}/localization/{suffix}',
                lambda m, st=static: self.on_amcl_tf(m, st), LATCHED if static else 100)
        self.create_subscription(PoseStamped, f'/qualisys/{self.robot}/pose', self.on_mocap, 100)
        self.create_subscription(Odometry, self.sensors+'/odom_enc', self.on_odom, qos_profile_sensor_data)
        for name, key in (('f_scan','front'), ('b_scan','rear')):
            self.create_subscription(LaserScan, self.sensors+'/'+name,
                lambda m, k=key: self.on_scan(m, k), qos_profile_sensor_data)
        self.create_subscription(GoalStatusArray, self.sensors+'/move_base/status', self.on_native, 10)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, LATCHED)
        self.create_subscription(String, '/'+self.robot+'/localization/comparison', self.on_amcl_report, LATCHED)
        self.create_subscription(Twist, 'cmd_vel_guarded', self.on_command, 10)
        self.backend = ActionClient(self, NavigateToPose, 'nav2_navigate_to_pose')
        self.server = ActionServer(self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.execute, goal_callback=self.accept_goal, cancel_callback=self.cancel_goal)
        self.create_service(Trigger, 'cancel', self.cancel_service)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.5, self.poll_states)
        self.create_timer(0.2, self.publish_status)

    def fresh(self, message, age=0.5):
        return abs(self.get_clock().now().nanoseconds*1e-9-stamp(message)) <= age

    def on_tf(self, message, static):
        edges = []
        for t in message.transforms:
            if (t.header.frame_id, t.child_frame_id) == ('map', 'mocap'):
                if static:
                    self.calibration.set_transform_static(t, 'hardware')
                    self.static_edges['mocap'] = t
                    self.private.set_transform_static(t, 'hardware')
            elif local_transform(t, self.robot):
                if static:
                    self.static_edges[t.child_frame_id] = t
                    self.private.set_transform_static(t, 'hardware')
                else:
                    self.private.set_transform(t, 'hardware')
                    edges.append(t)
        if static and self.static_edges:
            self.static_pub.publish(TFMessage(transforms=list(self.static_edges.values())))
        elif edges:
            self.tf_pub.publish(TFMessage(transforms=edges))

    def on_amcl_tf(self, message, static):
        for t in message.transforms:
            if static:
                self.amcl.set_transform_static(t, 'amcl')
            else:
                self.amcl.set_transform(t, 'amcl')
                if t.header.frame_id == 'mocap' and t.child_frame_id == self.robot+'/localization/odom' and self.fresh(t, 1.):
                    self.received['amcl'] = time.monotonic()

    def on_mocap(self, message):
        if not valid_pose(message) or not self.fresh(message):
            return
        when = stamp(message)
        if self.mocap and when <= self.mocap[-1][0]:
            return
        p = message.pose.position
        self.mocap.append((when, p.x, p.y, yaw(message.pose.orientation)))
        self.received['mocap'] = time.monotonic()

    def on_odom(self, message):
        if not self.fresh(message) or message.child_frame_id.strip('/') != self.base:
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 0.01:
            return
        velocity = message.twist.twist
        if not all(math.isfinite(v) for v in (velocity.linear.x,velocity.linear.y,velocity.linear.z,
                velocity.angular.x,velocity.angular.y,velocity.angular.z)):
            return
        self.encoder = message
        self.received['encoder'] = time.monotonic()
        output = copy.deepcopy(message)
        output.header.frame_id = self.odom_frame
        self.odom_pub.publish(output)
        edge = planar_transform(self.odom_frame, self.base, (p.x,p.y,yaw(q)), message.header.stamp)
        self.private.set_transform(edge, 'encoder')
        self.tf_pub.publish(TFMessage(transforms=[edge]))

    def on_scan(self, message, key):
        if not self.fresh(message) or not message.header.frame_id.startswith(self.robot+'/') or not message.ranges:
            return
        if (not all(math.isfinite(v) for v in (message.angle_min,message.angle_increment,message.range_min,message.range_max))
                or message.angle_increment == 0 or message.range_min < 0 or message.range_max <= message.range_min):
            return
        if not any(r == math.inf or (math.isfinite(r) and message.range_min <= r <= message.range_max)
                   for r in message.ranges):
            return
        self.scan_pubs[key].publish(message)
        self.scan_frames[key] = message.header.frame_id
        self.received[key] = time.monotonic()

    def on_native(self, message):
        if self.fresh(message, 1.):
            self.received['native'] = time.monotonic()
            self.native_busy = any(s.status in (0,1,6,7) for s in message.status_list)

    def on_map(self, message):
        if message.header.frame_id == 'mocap' and message.data:
            self.map = message

    def on_amcl_report(self, message):
        try:
            report = json.loads(message.data)
            self.amcl_map_matches = (bool(self.map_yaml) and report.get('initialized_from_mocap_once',False) and
                Path(report['map_yaml']).resolve() == Path(self.map_yaml).resolve())
            self.received['amcl_report'] = time.monotonic()
        except (ValueError,KeyError,TypeError):
            self.amcl_map_matches = False

    def on_command(self, message):
        if all(math.isfinite(v) for v in (message.linear.x, message.linear.y, message.angular.z)):
            self.command, self.command_at = message, time.monotonic()

    def poll_states(self):
        now = time.monotonic()
        for name, client in self.state_clients.items():
            pending = self.state_futures.get(name)
            if pending:
                future, requested = pending
                if future.done():
                    try:
                        self.states[name] = (future.result().current_state.id == 3, now)
                    except Exception:
                        self.states[name] = (False, now)
                    del self.state_futures[name]
                elif now-requested > 2.:
                    future.cancel()
                    del self.state_futures[name]
                    self.states[name] = (False, now)
            elif client.service_is_ready():
                self.state_futures[name] = (client.call_async(GetState.Request()), now)

    def update_localization(self):
        if not self.encoder:
            return
        when = stamp(self.encoder)
        pose = None
        if self.source == 'mocap':
            pose = interpolate(list(self.mocap), when)
        else:
            try:
                t = self.amcl.lookup_transform('mocap', self.base,
                    rclpy.time.Time.from_msg(self.encoder.header.stamp))
                pose = (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))
            except TransformException:
                pass
        if pose is None:
            return
        p, q = self.encoder.pose.pose.position, self.encoder.pose.pose.orientation
        edge = planar_transform('mocap', self.odom_frame, map_to_odom(pose, (p.x,p.y,yaw(q))),
                                self.encoder.header.stamp)
        self.private.set_transform(edge, 'localization')
        self.tf_pub.publish(TFMessage(transforms=[edge]))
        self.localized = (when, pose)
        display = PoseStamped()
        display.header.frame_id, display.header.stamp = 'mocap', self.encoder.header.stamp
        display.pose.position.x, display.pose.position.y = float(pose[0]), float(pose[1])
        display.pose.orientation.z, display.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        self.pose_pub.publish(display)

    def not_ready(self):
        if self.latched_fault:
            return self.latched_fault+'; Stack neu starten'
        now = time.monotonic()
        for key, label, age in (('encoder','Encoder-Odometrie',.5), ('front','Frontlaser',.5),
                ('rear','Rücklaser',.5), (self.source, self.source.upper(),.5),
                ('native','MiR-Navigationsstatus (Hardwaretreiber aktualisieren / neu starten)',1.)):
            if now-self.received.get(key, -100) > age:
                return 'Warte auf '+label
        if self.native_busy:
            return 'Interne MiR-Navigation ist aktiv; zuerst dort abbrechen'
        if self.source == 'amcl' and (not self.amcl_map_matches or now-self.received.get('amcl_report',-100) > 3.):
            return 'AMCL im Tab Lokalisierung auf derselben Karte starten'
        if self.localized is None or abs(self.get_clock().now().nanoseconds*1e-9-self.localized[0]) > .5:
            return 'Warte auf zeitlich passende Lokalisierung'
        for key in ('front','rear'):
            if not self.private.can_transform(self.base, self.scan_frames.get(key,''), rclpy.time.Time()):
                return 'Warte auf Scanner-TF'
        if self.map is None:
            return 'Warte auf Karte'
        for name in self.managed:
            active, received = self.states.get(name, (False, 0))
            if not active or now-received > 2.:
                return 'Warte auf Nav2: '+name
        if not self.backend.server_is_ready():
            return 'Warte auf Nav2-Aktionsserver'
        return ''

    def target(self, request):
        message = request.pose
        if request.behavior_tree:
            raise ValueError('Eigene Behavior Trees sind für diesen Fahrzugang nicht freigegeben')
        p, q = message.pose.position, message.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .01:
            raise ValueError('Ungültige Zielpose')
        frame = message.header.frame_id.strip('/')
        if not frame:
            raise ValueError('Ziel benötigt einen Koordinatenrahmen')
        output = copy.deepcopy(message)
        if frame != 'mocap':
            buffer = self.calibration if frame == 'map' else self.private
            transform = buffer.lookup_transform('mocap', frame, rclpy.time.Time())
            output = do_transform_pose_stamped(output, transform)
        output.header.frame_id, output.header.stamp = 'mocap', self.get_clock().now().to_msg()
        return output

    def accept_goal(self, request):
        reason = self.not_ready()
        if self.busy or time.monotonic() < self.reject_until:
            reason = 'Vorherige Fahrt noch nicht beendet'
        try:
            self.target(request)
        except (ValueError, TransformException) as exc:
            reason = str(exc)
        if reason:
            self.status_text = 'Ziel abgelehnt: '+reason
            self.get_logger().warning(self.status_text)
            return GoalResponse.REJECT
        self.busy = True
        self.cancel_requested = False
        self.failure = ''
        return GoalResponse.ACCEPT

    def halt(self):
        self.motion_enabled = False
        self.last_velocity = (0., 0.)
        self.hold_zero_until = time.monotonic()+0.3
        self.publish_velocity(0., 0.)

    def cancel_goal(self, _handle):
        self.cancel_requested = True
        self.halt()
        return CancelResponse.ACCEPT

    def cancel_service(self, _request, response):
        self.reject_until = time.monotonic()+0.5
        if self.busy:
            self.cancel_requested = True
            self.status_text = 'Abbruch angefordert'
            self.halt()
        response.success, response.message = True, 'Abbruch angefordert; Fahrbefehle gesperrt'
        return response

    def pause(self, delay):
        future = Future(executor=self.executor)
        self.waiters.append((time.monotonic()+delay, future))
        return future

    async def wait(self, future, timeout):
        deadline = time.monotonic()+timeout
        while not future.done():
            if self.shutting_down:
                raise RuntimeError('Navigationsstack wird beendet')
            if time.monotonic() > deadline:
                raise TimeoutError('Nav2 antwortet nicht rechtzeitig')
            await self.pause(.05)
        return future.result()

    async def execute(self, handle):
        self.front_handle = handle
        result = NavigateToPose.Result()
        backend_result = None
        sending = None
        try:
            goal = NavigateToPose.Goal(pose=self.target(handle.request))
            def feedback(message):
                self.distance_remaining = float(message.feedback.distance_remaining)
                if handle.is_active:
                    handle.publish_feedback(message.feedback)
            sending = self.backend.send_goal_async(goal, feedback_callback=feedback)
            self.backend_handle = await self.wait(sending, 3.)
            if not self.backend_handle.accepted:
                raise RuntimeError('Nav2 hat das Ziel abgelehnt')
            backend_result = self.backend_handle.get_result_async()
            self.command = None
            reason = self.not_ready()
            if reason:
                self.failure = reason
            self.motion_enabled = not (self.failure or self.cancel_requested or handle.is_cancel_requested)
            started = time.monotonic()
            self.status_text = 'Fahrt läuft'
            while not backend_result.done():
                if self.failure or self.cancel_requested or handle.is_cancel_requested or time.monotonic()-started > self.settings['goal_timeout']:
                    if not self.failure and not (self.cancel_requested or handle.is_cancel_requested):
                        self.failure = 'Zielzeitlimit überschritten'
                    self.halt()
                    cancellation = await self.wait(self.backend_handle.cancel_goal_async(), 3.)
                    if not cancellation.goals_canceling and not backend_result.done():
                        raise RuntimeError('Nav2 bestätigt den Abbruch nicht')
                    await self.wait(backend_result, 4.)
                    break
                await self.pause(.05)
            finished = backend_result.result()
            result = finished.result
            if self.failure:
                result.error_code, result.error_msg = 1, self.failure
                handle.abort()
            elif self.cancel_requested or handle.is_cancel_requested:
                # A Trigger stop is an abort, since the action client did not request cancellation.
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    result.error_code, result.error_msg = 1, 'Fahrt abgebrochen'
                    handle.abort()
            elif finished.status == GoalStatus.STATUS_SUCCEEDED and result.error_code == 0:
                handle.succeed()
            else:
                handle.abort()
            self.status_text = 'Ziel erreicht' if handle.status == GoalStatus.STATUS_SUCCEEDED else ('Fahrt beendet: '+(result.error_msg or 'abgebrochen'))
        except Exception as exc:
            self.halt()
            self.status_text = str(exc)
            if sending is not None and not sending.done():
                self.latched_fault = 'Nav2-Zielannahme unbestätigt'
                def cancel_late(future):
                    try:
                        late = future.result()
                        if late.accepted:
                            late.cancel_goal_async()
                    except Exception:
                        pass
                sending.add_done_callback(cancel_late)
            if self.backend_handle and self.backend_handle.accepted and (backend_result is None or not backend_result.done()):
                self.backend_handle.cancel_goal_async()
                self.latched_fault = 'Nav2-Abbruch unbestätigt'
            result.error_code, result.error_msg = 1, str(exc)
            if handle.is_active:
                handle.abort()
        finally:
            self.halt()
            self.busy = False
            self.backend_handle = self.front_handle = None
            self.distance_remaining = None
        return result

    def publish_velocity(self, linear, angular):
        output = TwistStamped()
        output.header.stamp, output.header.frame_id = self.get_clock().now().to_msg(), self.base
        output.twist.linear.x, output.twist.angular.z = float(linear), float(angular)
        self.velocity_pub.publish(output)

    def tick(self):
        now = time.monotonic()
        for deadline, future in list(self.waiters):
            if now >= deadline:
                if not future.done():
                    future.set_result(None)
                self.waiters.remove((deadline, future))
        self.update_localization()
        if self.motion_enabled:
            reason = self.not_ready()
            if reason:
                self.failure = reason
                self.halt()
            else:
                cmd = self.command if now-self.command_at < .3 else None
                vx = min(self.settings['max_speed'], max(-self.settings['max_reverse_speed'], cmd.linear.x)) if cmd else 0.
                wz = min(self.settings['max_angular_speed'], max(-self.settings['max_angular_speed'], cmd.angular.z)) if cmd else 0.
                # The upstream velocity smoother enforces acceleration/deceleration.
                # This final boundary also clamps speed, and bypasses ramps for emergency stops.
                self.publish_velocity(vx, wz)
        elif now < self.hold_zero_until:
            self.publish_velocity(0., 0.)

    def publish_status(self):
        reason = self.not_ready()
        self.status_pub.publish(String(data=json.dumps(dict(robot=self.robot, source=self.source,
            ready=not reason, busy=self.busy, motion_enabled=self.motion_enabled,
            state=(self.status_text if self.busy else reason or 'Bereit · '+self.status_text),
            last_result=self.status_text, distance_remaining=self.distance_remaining))))

    def close(self):
        if self.busy or self.motion_enabled:
            self.halt()
        if self.backend_handle and self.backend_handle.accepted:
            self.backend_handle.cancel_goal_async()
        self.server.destroy()
        self.backend.destroy()
        self.destroy_node()


def main():
    def stop(_signal, _frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NavigationSession()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutting_down = True
        if node.busy:
            node.failure = 'Navigationsstack wird beendet'
            node.halt()
            deadline = time.monotonic()+1.
            while node.busy and time.monotonic() < deadline:
                rclpy.spin_once(node,timeout_sec=.05)
        node.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
