"""Base localization must work without Mocap and switch without changing TF parents."""

import time

import pytest
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from mir_launch_hardware.mir_pose_simple import AutomaticLocalization, MirPoseSimple


def pose(x, stamp=100.0, frame='map'):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    msg.pose.position.x = x
    msg.pose.position.z = 0.84
    msg.pose.orientation.w = 1.0
    return msg


def test_native_start_mocap_priority_timeout_and_recovery():
    selection = AutomaticLocalization(1.0)
    assert selection.select(10.0) == ('waiting', None)
    assert selection.observe('mir', pose(1.0), 10.0, 100.0)
    assert selection.select(10.0)[0] == 'mir'
    assert selection.observe('mocap', pose(9.0), 10.1, 100.0)
    assert selection.observe('mir', pose(2.0), 10.9, 100.0)
    assert selection.select(10.9)[1].pose.position.x == 9.0
    assert selection.select(11.2)[0] == 'mir'
    assert selection.select(12.0) == ('waiting', None)
    assert selection.observe('mocap', pose(8.0), 12.1, 100.0)
    assert selection.select(12.1)[0] == 'mocap'


@pytest.mark.parametrize('bad', [
    pose(9.0, frame='mocap'), pose(9.0, stamp=90.0), pose(9.0, stamp=110.0),
    pose(float('nan')), pose(float('inf')),
])
def test_invalid_or_delayed_mocap_does_not_replace_native(bad):
    selection = AutomaticLocalization(1.0)
    selection.observe('mir', pose(1.0), 10.0, 100.0)
    assert not selection.observe('mocap', bad, 10.0, 100.0)
    assert selection.select(10.0)[0] == 'mir'


def test_invalid_orientation_is_rejected_and_held_pose_is_accepted():
    selection = AutomaticLocalization(1.0)
    bad = pose(9.0)
    bad.pose.orientation.w = 0.0
    assert not selection.observe('mocap', bad, 10.0, 100.0)
    # Freeze re-publishes the same full pose with a fresh timestamp.
    held = pose(9.0, stamp=110.0)
    assert selection.observe('mocap', held, 20.0, 110.0)
    assert selection.select(20.1)[1].pose.position.z == 0.84


def test_ros_source_handoff_keeps_one_map_to_base_edge():
    rclpy.init(args=[
        '--ros-args', '-r', '__ns:=/mur620c',
        '-p', 'localization_type:=auto', '-p', 'publish_robot_tf:=true',
        '-p', 'mocap_topic:=/qualisys_map/mur620c/pose',
        '-p', 'localization_timeout:=0.3',
    ])
    selector = MirPoseSimple()
    probe = Node('localization_test_probe', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(selector)
    executor.add_node(probe)
    transforms = []
    native = probe.create_publisher(Pose, '/mur620c/robot_pose', 10)
    mocap = probe.create_publisher(PoseStamped, '/qualisys_map/mur620c/pose', 10)
    probe.create_subscription(TFMessage, '/tf', transforms.append, 100)

    def until(predicate, *, send_native=False, send_mocap=False):
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            stamp = probe.get_clock().now().nanoseconds * 1e-9
            if send_native:
                native.publish(pose(1.0, stamp).pose)
            if send_mocap:
                mocap.publish(pose(9.0, stamp))
            executor.spin_once(timeout_sec=0.01)
            if predicate():
                return
        pytest.fail('Localization did not reach the expected source / transform')

    def latest_x():
        return transforms[-1].transforms[-1].transform.translation.x if transforms else None

    try:
        until(lambda: latest_x() == 1.0, send_native=True)
        assert selector.localization_source == 'mir'
        until(lambda: latest_x() == 9.0, send_native=True, send_mocap=True)
        assert selector.localization_source == 'mocap'
        until(lambda: latest_x() == 1.0, send_native=True)
        assert selector.localization_source == 'mir'
        until(lambda: selector.localization_source == 'waiting')
        # Already-running / returning Mocap is selected without a native pose.
        until(lambda: latest_x() == 9.0, send_mocap=True)
        assert selector.localization_source == 'mocap'
        assert all(t.header.frame_id == 'map' and t.child_frame_id == 'mur620c/base_footprint'
                   for msg in transforms for t in msg.transforms)
    finally:
        executor.shutdown()
        selector.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
