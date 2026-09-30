import math

import pytest
from geometry_msgs.msg import PoseStamped

from match_mocap_ros2.map_transform import MapToMocap, checked_map_transform


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
