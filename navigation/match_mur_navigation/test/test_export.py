import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
import yaml

from match_mur_navigation.save_map import save
from match_mur_navigation.session import LATCHED


def spin_until(node, predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert predicate(), 'Timed out waiting for ROS operation'


def test_nav2_export_reload_and_overwrite_protection(tmp_path):
    rclpy.init(args=[])
    namespace = '/export_' + uuid.uuid4().hex[:8]
    node = rclpy.create_node('export_probe', namespace=namespace)
    pub = node.create_publisher(OccupancyGrid, namespace+'/snapshot', LATCHED)
    msg = OccupancyGrid()
    msg.header.frame_id = 'mocap'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.info.resolution = 0.02
    msg.info.width, msg.info.height = 3, 2
    msg.info.origin.position.x, msg.info.origin.position.y = -2.4, 1.2
    msg.info.origin.orientation.w = 1.0
    msg.data = [0, 100, -1, 100, -1, 0]
    pub.publish(msg)
    process = None
    log = (tmp_path/'map_server.log').open('w')
    try:
        outputs = save(node, namespace+'/snapshot', tmp_path/'lab.v1',
                       dict(robot='mur620d', mode='mocap', settings={}))
        assert all(p.is_file() for p in outputs)
        companion = yaml.safe_load(outputs[2].read_text())
        assert companion['frame'] == 'mocap'
        assert yaml.safe_load(outputs[0].read_text())['image'] == 'lab.v1.pgm'
        with pytest.raises(FileExistsError):
            save(node, namespace+'/snapshot', tmp_path/'lab.v1',
                 dict(robot='mur620d', mode='mocap', settings={}))
        process = subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
            '-r', '__ns:='+namespace, '-p', 'yaml_filename:='+str(outputs[0]), '-p', 'frame_id:=mocap'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        client = node.create_client(ChangeState, namespace+'/map_server/change_state')
        spin_until(node, client.service_is_ready)
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            spin_until(node, future.done)
            assert future.result().success
        maps = []
        node.create_subscription(OccupancyGrid, namespace+'/map', maps.append, LATCHED)
        spin_until(node, lambda: bool(maps))
        loaded = maps[-1]
        assert loaded.header.frame_id == 'mocap'
        assert list(loaded.data) == list(msg.data)
        assert loaded.info.resolution == pytest.approx(msg.info.resolution)
        assert loaded.info.origin.position.x == pytest.approx(-2.4)
        assert loaded.info.origin.position.y == pytest.approx(1.2)
    finally:
        if process:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()
