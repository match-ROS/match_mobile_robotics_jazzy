"""Launch both real backends against synthetic sensors on a hardware-isolated domain."""
import json
import math
import os
import signal
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data
import yaml
from PyQt5 import QtCore, QtWidgets

from match_mur_navigation.session import LATCHED
from match_mur_navigation.settings import DEFAULTS
from match_mur_navigation.gui import NavigationMonitor


@pytest.mark.parametrize('mode', ['mocap', 'slam'])
def test_launch_mapping_stop_and_tracking_recovery(mode, tmp_path):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    config = tmp_path/'params.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': dict(
        DEFAULTS, resolution=0.05, map_update_interval=0.1)}}))
    rclpy.init(args=[])
    node = rclpy.create_node('mapping_synthetic_sensor')
    robot, ns = 'mur620d', '/mur620d/mapping'
    poses = node.create_publisher(PoseStamped, '/qualisys/'+robot+'/pose', 100)
    odom = node.create_publisher(Odometry, '/'+robot+'/odom', qos_profile_sensor_data)
    scans = {name: node.create_publisher(LaserScan, '/'+robot+'/'+name, qos_profile_sensor_data)
             for name in ('f_scan', 'b_scan', 'scan')}
    static = node.create_publisher(TFMessage, '/tf_static', LATCHED)
    edges = []
    for parent, child, x, y, yaw in (
        ('base_footprint', 'base_link', 0, 0, 0),
        ('base_link', 'front_laser_link', 0.3, 0.2, 0),
        ('base_link', 'back_laser_link', -0.3, -0.2, math.pi),
        ('base_link', 'virtual_laser_link', 0, 0, 0)):
        t = TransformStamped(child_frame_id=robot+'/'+child)
        t.header.frame_id = robot+'/'+parent
        t.transform.translation.x, t.transform.translation.y = float(x), float(y)
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        edges.append(t)
    maps, statuses, global_roots = [], [], []
    node.create_subscription(OccupancyGrid, ns+'/map', maps.append, LATCHED)
    node.create_subscription(String, ns+'/status', lambda m: statuses.append(json.loads(m.data)), LATCHED)
    node.create_subscription(TFMessage, '/tf', lambda m: global_roots.extend(m.transforms), 100)
    log = (tmp_path/'launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'match_mur_navigation', 'mapping.launch.py',
        'robot:='+robot, 'mode:='+mode, 'params_file:='+str(config)],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    monitor = None

    def pump(seconds, with_pose=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path/'launch.log').read_text()
            now = node.get_clock().now().nanoseconds
            static.publish(TFMessage(transforms=edges))
            if with_pose:
                p = PoseStamped()
                p.header.frame_id = 'mocap'
                p.header.stamp = rclpy.time.Time(nanoseconds=now).to_msg()
                p.pose.position.x, p.pose.position.y, p.pose.orientation.w = 1.0, 2.0, 1.0
                poses.publish(p)
            o = Odometry()
            o.header.frame_id, o.child_frame_id = robot+'/odom', robot+'/base_footprint'
            o.header.stamp = rclpy.time.Time(nanoseconds=now).to_msg()
            o.pose.pose.orientation.w = 1.0
            odom.publish(o)
            for name, frame in (('f_scan', 'front_laser_link'), ('b_scan', 'back_laser_link'),
                                ('scan', 'virtual_laser_link')):
                s = LaserScan()
                s.header.frame_id = robot+'/'+frame
                s.header.stamp = rclpy.time.Time(nanoseconds=now-50000000).to_msg()
                s.angle_min, s.angle_max, s.angle_increment = -1.5, 1.5, 0.01
                s.range_min, s.range_max, s.time_increment = 0.05, 20.0, 0.0
                # A corner supplies enough structure for actual SLAM scan matching.
                s.ranges = [min(2/max(0.001, math.cos(-1.5+i*0.01)),
                                2/max(0.001, abs(math.sin(-1.5+i*0.01)))) for i in range(301)]
                scans[name].publish(s)
            rclpy.spin_once(node, timeout_sec=0.03)
            app.processEvents()
            time.sleep(0.01)

    def end_session():
        if monitor is not None:
            results = []
            monitor.session_end_result.connect(lambda *args: results.append(args))
            monitor.request_end(robot)
            deadline = time.monotonic()+4
            while not results and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                app.processEvents()
            assert results and results[-1][0:2] == (robot, True)
        else:
            client = node.create_client(Trigger, ns+'/end_session')
            assert client.wait_for_service(timeout_sec=3)
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic()+4
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            assert future.done() and future.result().success
        process.wait(timeout=7)
        assert process.returncode == 0, (tmp_path/'launch.log').read_text()
        assert node.context.ok()
    try:
        deadline = time.monotonic()+20
        while not maps and time.monotonic() < deadline:
            pump(0.2)
        assert maps, (tmp_path/'launch.log').read_text()
        assert maps[-1].header.frame_id == ('mocap' if mode == 'mocap' else robot+'/mapping/map')
        assert max(maps[-1].data) >= 65
        assert min(maps[-1].data) == -1
        assert not global_roots
        for endpoint in node.get_publishers_info_by_topic('/tf'):
            assert endpoint.node_namespace != ns
        for endpoint in node.get_publishers_info_by_topic('/map'):
            assert endpoint.node_namespace != ns
        if mode == 'mocap':
            def occupied_near(x, y):
                m = maps[-1]
                ix = math.floor((x-m.info.origin.position.x)/m.info.resolution)
                iy = math.floor((y-m.info.origin.position.y)/m.info.resolution)
                return max(m.data[(iy+j)*m.info.width+ix+i] for i in (-1, 0, 1)
                           for j in (-1, 0, 1)) >= 65
            assert occupied_near(3.3, 2.2) # Front scanner extrinsic and absolute Mocap position.
            assert occupied_near(-1.3, 1.8) # Rear scanner rotated by pi.
            pump(0.5)
            pump(1.1, with_pose=False)
            integrated = statuses[-1]['integrated']
            discarded = statuses[-1]['discarded']
            pump(0.7, with_pose=False)
            assert statuses[-1]['integrated'] == integrated
            assert statuses[-1]['discarded'] > discarded
            pump(0.8)
            assert statuses[-1]['integrated'] > integrated
        client = node.create_client(SetBool, ns+'/set_recording')
        assert client.wait_for_service(timeout_sec=3)
        future = client.call_async(SetBool.Request(data=False))
        deadline = time.monotonic()+3
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert future.done() and future.result().success
        pump(0.5)
        assert statuses[-1]['recording'] is False
        data = list(maps[-1].data)
        pump(0.5)
        assert list(maps[-1].data) == data
        if mode == 'mocap':
            # The actual Qt worker must retain the map independently of backend lifetime.
            monitor = NavigationMonitor()
            changes = []
            monitor.map_changed.connect(lambda *value: changes.append(value))
            monitor.start()
            monitor.reset(robot, statuses[-1]['session_id'])
            deadline = time.monotonic()+6
            while not changes and time.monotonic() < deadline:
                pump(0.2)
            assert changes
            cache = []
            node.create_subscription(OccupancyGrid, monitor.cache_topic(robot), cache.append, LATCHED)
            pump(0.3)
            assert cache
            end_session()
            late_cache = []
            node.create_subscription(OccupancyGrid, monitor.cache_topic(robot), late_cache.append, LATCHED)
            deadline = time.monotonic()+3
            while not late_cache and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                app.processEvents()
            assert late_cache and list(late_cache[-1].data) == list(cache[-1].data)
        else:
            end_session()
    finally:
        if monitor:
            monitor.shutdown()
            assert monitor.wait(3000)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=7)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()
