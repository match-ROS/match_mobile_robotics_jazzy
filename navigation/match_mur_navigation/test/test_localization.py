"""Only the new comparison math and the real AMCL/map/TF integration."""
import json
import math
import os
import signal
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
import yaml

from match_mur_navigation.comparison import distribution, errors, interpolate
from match_mur_navigation.session import LATCHED


def test_timestamp_interpolation_wrap_and_error_statistics():
    a = math.radians(179)
    b = math.radians(-179)
    values = [(1.0, 0., 2., a), (1.05, 2., 4., b)]
    matched = interpolate(values, 1.025)
    assert matched[:2] == pytest.approx((1., 3.))
    assert abs(abs(matched[2])-math.pi) < 1e-8
    assert interpolate(values, 0.9) is None
    assert interpolate(values, 1.1) is None
    assert interpolate([(1., 0., 0., 0.), (2., 1., 1., 1.)], 1.5) is None
    assert errors((3., 4., b), (0., 0., a)) == pytest.approx((3., 4., 5., 2.))
    assert distribution([]) is None
    assert distribution([1., 2., 3.]) == pytest.approx(dict(rmse=math.sqrt(14/3), median=2., p95=2.9, maximum=3.))


def test_real_amcl_private_tf_one_seed_and_comparison(tmp_path):
    # A synthetic square room exercises the real lifecycle/map server and AMCL.
    # The ROS domain is forced to localhost by conftest; no hardware is touched.
    size, resolution = 160, 0.05
    image = bytearray([254])*(size*size)
    for row in range(size):
        for col in range(size):
            x, y = -4+(col+.5)*resolution, 4-(row+.5)*resolution
            if abs(abs(x)-3) < .06 or abs(abs(y)-3) < .06:
                image[row*size+col] = 0
    (tmp_path/'room.pgm').write_bytes(f'P5\n{size} {size}\n255\n'.encode()+image)
    path = tmp_path/'room.yaml'
    path.write_text(yaml.safe_dump(dict(image='room.pgm', mode='trinary', resolution=resolution,
        origin=[-4., -4., 0.], negate=0, occupied_thresh=.65, free_thresh=.196)))
    path.with_suffix('.mapping.yaml').write_text(yaml.safe_dump(dict(frame='mocap', mode='mocap')))
    rclpy.init(args=[])
    node = rclpy.create_node('localization_synthetic_sensor')
    robot, ns = 'mur620c', '/mur620c/localization'
    sensor_namespace = '/mur620c/test_inputs'
    poses = node.create_publisher(PoseStamped, '/qualisys/'+robot+'/pose', 100)
    odom = node.create_publisher(Odometry, sensor_namespace+'/odom_enc', qos_profile_sensor_data)
    scans = node.create_publisher(LaserScan, sensor_namespace+'/f_scan', qos_profile_sensor_data)
    static = node.create_publisher(TFMessage, sensor_namespace+'/tf_static', LATCHED)
    input_roots = node.create_publisher(TFMessage, sensor_namespace+'/tf', 10)
    roots = node.create_publisher(TFMessage, '/tf', 10)
    edges = []
    for parent, child in [('base_footprint', 'base_link'), ('base_link', 'front_laser_link')]:
        t = TransformStamped(child_frame_id=robot+'/'+child)
        t.header.frame_id, t.transform.rotation.w = robot+'/'+parent, 1.0
        edges.append(t)
    private, global_tf, estimates, seeds, reports, maps = [], [], [], [], [], []
    node.create_subscription(TFMessage, ns+'/tf', lambda m: private.extend(m.transforms), 100)
    node.create_subscription(TFMessage, '/tf', lambda m: global_tf.extend(m.transforms), 100)
    node.create_subscription(PoseWithCovarianceStamped, ns+'/initialpose', seeds.append, 10)
    node.create_subscription(PoseWithCovarianceStamped, ns+'/amcl_pose', estimates.append, 100)
    node.create_subscription(String, ns+'/comparison', lambda m: reports.append(json.loads(m.data)), LATCHED)
    node.create_subscription(OccupancyGrid, ns+'/map', maps.append, LATCHED)
    stationary = node.create_client(SetBool, ns+'/set_stationary_updates')
    enabled = None
    late_poses = []
    log_path = tmp_path/'launch.log'
    log = log_path.open('w')
    environment = dict(os.environ)
    environment.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    environment.pop('FASTDDS_DEFAULT_PROFILES_FILE', None)
    map_process = subprocess.Popen(['ros2', 'launch', 'match_mur_navigation', 'localization.launch.py',
        'robot:='+robot, 'map:='+str(path), 'localize:=false'], env=environment,
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    map_deadline = time.monotonic()+5
    while not maps and time.monotonic() < map_deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    process = subprocess.Popen(['ros2', 'launch', 'match_mur_navigation', 'localization.launch.py',
        'robot:='+robot, 'map:='+str(path), 'localize:=true', 'stationary_updates:=false',
        'load_map:=false',
        'sensor_namespace:='+sensor_namespace,
        'output_directory:='+str(tmp_path/'results')], env=environment,
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic()+26
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            stamp = node.get_clock().now().to_msg()
            static.publish(TFMessage(transforms=edges))
            # A poisoned hardware root must never enter private localization TF.
            root = TransformStamped(child_frame_id=robot+'/base_footprint')
            root.header.frame_id, root.header.stamp = 'hardware_map', stamp
            root.transform.rotation.w = 1.0
            roots.publish(TFMessage(transforms=[root]))
            input_roots.publish(TFMessage(transforms=[root]))
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = 'mocap', stamp
            pose.pose.orientation.w = 1.0
            poses.publish(pose)
            o = Odometry()
            o.header.frame_id, o.header.stamp, o.child_frame_id = robot+'/odom', stamp, robot+'/base_footprint'
            o.pose.pose.orientation.w = 1.0
            odom.publish(o)
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = robot+'/front_laser_link', stamp
            scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/180
            scan.range_min, scan.range_max = .05, 10.0
            scan.ranges = [min(3/max(1e-6, abs(math.cos(-math.pi+i*math.pi/180))),
                               3/max(1e-6, abs(math.sin(-math.pi+i*math.pi/180)))) for i in range(361)]
            scans.publish(scan)
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=.01)
            if (enabled is None and reports and
                    (reports[-1].get('amcl_pose_age_seconds') or 0) > 5 and stationary.service_is_ready()):
                # No movement means no new poses, but the localization is valid.
                assert reports[-1]['live']['amcl']
                assert not reports[-1]['live']['amcl_updates']
                assert reports[-1]['samples'] == 0
                node.create_subscription(PoseWithCovarianceStamped, ns+'/amcl_pose', late_poses.append, LATCHED)
                enabled = stationary.call_async(SetBool.Request(data=True))
            if reports and reports[-1]['samples'] >= 3:
                break
        assert maps and maps[-1].header.frame_id == 'mocap', log_path.read_text()
        assert len(seeds) == 1, log_path.read_text()
        assert estimates, log_path.read_text()
        assert reports[-1]['samples'] >= 3, log_path.read_text()
        assert enabled is not None and enabled.done() and enabled.result().success
        assert late_poses, 'A later-opened RViz must receive the retained AMCL pose.'
        assert len(reports[-1]['stationary_update_changes']) == 1
        assert reports[-1]['stationary_updates']
        assert reports[-1]['odom_topic'] == sensor_namespace+'/odom_enc'
        assert reports[-1]['position_m']['rmse'] < .4
        assert reports[-1]['yaw_deg']['rmse'] < 10
        assert any(t.header.frame_id == 'mocap' for t in private)
        assert any(t.header.frame_id == robot+'/localization/odom' for t in private)
        assert not any(t.header.frame_id == 'hardware_map' for t in private)
        assert all(t.header.frame_id == 'hardware_map' for t in global_tf)
        assert node.count_publishers('/'+robot+'/cmd_vel') == 0
        assert node.count_publishers('/'+robot+'/initialpose') == 0
        output = reports[-1]['output']
        assert len(open(output+'.csv').readlines()) >= 4
        assert json.load(open(output+'.json'))['samples'] >= 3
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        map_survived = map_process.poll() is None
        if map_process.poll() is None:
            os.killpg(map_process.pid, signal.SIGINT)
            try:
                map_process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(map_process.pid, signal.SIGTERM)
                map_process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    assert 'Traceback' not in log_path.read_text(), log_path.read_text()
    assert map_survived, 'Stopping AMCL must not stop the reused map server.'
