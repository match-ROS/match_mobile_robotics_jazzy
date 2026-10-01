import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from match_mocap_ros2.map_transform import MapToMocap, checked_map_transform
from match_mocap_ros2.qualisys_ssh_bridge import (
    LocalizationHold, QualisysSshBridge, robot_tf_from_map_pose
)


LAUNCH = '''<launch><node pkg="tf" type="static_transform_publisher"
    name="map_to_mocap" args="48.2795 43.2532 0 1.290419 0 0 map mocap 1" /></launch>'''


def test_ros1_map_to_mocap_direction_and_pose():
    transform = MapToMocap.from_ros1_launch(LAUNCH)
    pose = PoseStamped()
    pose.header.frame_id = 'mocap'
    pose.header.stamp.sec = 123
    pose.pose.position.x = 1.0
    pose.pose.orientation.w = 1.0
    mapped = transform.apply(pose)
    assert mapped.header.frame_id == 'map'
    assert mapped.header.stamp.sec == 123
    assert mapped.pose.position.x == pytest.approx(48.2795 + math.cos(1.290419))
    assert mapped.pose.position.y == pytest.approx(43.2532 + math.sin(1.290419))
    assert mapped.pose.orientation.z == pytest.approx(math.sin(1.290419 / 2))
    assert mapped.pose.orientation.w == pytest.approx(math.cos(1.290419 / 2))
    assert pose.header.frame_id == 'mocap'
    assert pose.pose.position.x == 1.0


def test_wrong_transform_direction_is_rejected():
    with pytest.raises(ValueError, match='map mocap'):
        MapToMocap.from_ros1_launch(LAUNCH.replace('map mocap', 'mocap map'))


def test_nonplanar_transform_is_rejected():
    with pytest.raises(ValueError, match='planar'):
        MapToMocap.from_ros1_launch(LAUNCH.replace('1.290419 0 0', '1.290419 0.1 0'))


def test_stale_repository_calibration_is_rejected():
    old_launch = LAUNCH.replace('48.2795 43.2532 0 1.290419',
                                '38.2691 32.8942 0 3.1656')
    with pytest.raises(ValueError, match='verified calibration'):
        checked_map_transform(old_launch)
    assert checked_map_transform(LAUNCH) == MapToMocap(48.2795, 43.2532, 0, 1.290419)


def test_robot_tf_preserves_full_raw_qualisys_base_link_pose():
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 7
    pose.pose.position.x = 3.0
    pose.pose.position.y = 4.0
    pose.pose.position.z = 0.84
    pose.pose.orientation.x = 0.1
    pose.pose.orientation.y = 0.2
    pose.pose.orientation.z = 0.3
    pose.pose.orientation.w = 0.9

    for robot in ('mur620a', 'mur620b', 'mur620c', 'mur620d'):
        tf = robot_tf_from_map_pose(pose, robot)
        assert tf.header.frame_id == 'map'
        assert tf.header.stamp.sec == 7
        assert tf.child_frame_id == f'{robot}/base_footprint'
        assert tf.transform.translation.x == 3.0
        assert tf.transform.translation.y == 4.0
        assert tf.transform.translation.z == 0.84
        assert tf.transform.rotation == pose.pose.orientation

    pose.header.frame_id = 'mocap'
    with pytest.raises(ValueError, match='map-frame'):
        robot_tf_from_map_pose(pose, 'mur620a')


def test_only_raw_map_pose_drives_robot_tf():
    published = []
    transforms = []
    fake = SimpleNamespace(
        map_transform=MapToMocap(48.2795, 43.2532, 0.0, 1.290419),
        _map_publishers={
            '/qualisys_map/mur620a/pose': SimpleNamespace(publish=published.append),
            '/qualisys_map/mur620a/pose_smoothed': SimpleNamespace(publish=published.append),
        },
        publish_robot_tf=True,
        localization_hold=LocalizationHold(),
        body_tf_broadcaster=SimpleNamespace(sendTransform=transforms.append),
    )
    pose = PoseStamped()
    pose.header.frame_id = 'mocap'
    pose.pose.orientation.w = 1.0
    pose.pose.position.z = 0.84
    QualisysSshBridge.publish_map(fake, 'mur620a', pose)
    QualisysSshBridge.publish_map(fake, 'mur620a', pose, smoothed=True)
    assert len(published) == 2
    assert len(transforms) == 1
    assert transforms[0].header.frame_id == 'map'
    assert transforms[0].child_frame_id == 'mur620a/base_footprint'
    assert transforms[0].transform.translation.z == 0.84


def test_localization_hold_requires_fresh_pose_and_keeps_full_pose():
    hold = LocalizationHold()
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = 3.0
    pose.pose.position.z = 0.84
    pose.pose.orientation.w = 1.0

    assert not hold.set_frozen('mur620a', True, 10.0)[0]
    hold.observe('mur620a', pose, 10.0)
    assert hold.set_frozen('mur620a', True, 10.1)[0]
    pose.pose.position.x = 9.0
    hold.observe('mur620a', pose, 10.2)
    frozen = hold.frozen_pose('mur620a', Time(sec=11, nanosec=0))
    assert frozen.pose.position.x == 3.0
    assert frozen.pose.position.z == 0.84
    assert frozen.header.stamp.sec == 11
    assert not hold.set_frozen('mur620a', False, 11.0)[0]
    assert 'mur620a' in hold.frozen
    assert hold.set_frozen('mur620a', False, 10.3)[0]
    assert hold.frozen_pose('mur620a', None) is None


def test_frozen_map_topics_and_robot_tf_ignore_new_qtm_pose():
    published = []
    transforms = []
    hold = LocalizationHold()
    old = PoseStamped()
    old.header.frame_id = 'map'
    old.pose.position.x = 2.0
    old.pose.orientation.w = 1.0
    hold.observe('mur620a', old, 10.0)
    assert hold.set_frozen('mur620a', True, 10.1)[0]
    fake = SimpleNamespace(
        map_transform=MapToMocap(0.0, 0.0, 0.0, 0.0),
        _map_publishers={
            '/qualisys_map/mur620a/pose': SimpleNamespace(publish=published.append),
            '/qualisys_map/mur620a/pose_smoothed': SimpleNamespace(publish=published.append),
        },
        localization_hold=hold,
        publish_robot_tf=True,
        body_tf_broadcaster=SimpleNamespace(sendTransform=transforms.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: Time(sec=11, nanosec=0)
        )),
    )
    live = PoseStamped()
    live.header.frame_id = 'mocap'
    live.pose.position.x = 9.0
    live.pose.orientation.w = 1.0
    QualisysSshBridge.publish_map(fake, 'mur620a', live)
    QualisysSshBridge.publish_map(fake, 'mur620a', live, smoothed=True)
    assert not published and not transforms
    QualisysSshBridge.publish_frozen_localizations(fake)
    assert len(published) == 2
    assert all(pose.pose.position.x == 2.0 for pose in published)
    assert len(transforms) == 1
    assert transforms[0].transform.translation.x == 2.0
    assert transforms[0].header.stamp.sec == 11
