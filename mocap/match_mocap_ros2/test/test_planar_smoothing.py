import math

import pytest
from geometry_msgs.msg import PoseStamped

from match_mocap_ros2.planar_smoothing import PlanarMovingAverage


def pose(x, y, yaw_deg, stamp_ns):
    msg = PoseStamped()
    msg.header.frame_id = 'mocap'
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
    msg.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
    return msg


def yaw_degrees(msg):
    return math.degrees(2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w))


def test_mean_wraps_yaw_and_stamps_window_midpoint():
    mean = PlanarMovingAverage(0.2)
    mean.add(pose(1, 2, 179, 1_000_000_000), 0.0)
    result = mean.add(pose(3, 4, -179, 1_100_000_000), 0.1)
    assert (result.pose.position.x, result.pose.position.y) == (2, 3)
    assert abs(yaw_degrees(result)) == pytest.approx(180)
    assert result.header.stamp.sec == 1
    assert result.header.stamp.nanosec == 50_000_000
    assert result.pose.position.z == 0
    assert result.header.frame_id == 'mocap'


def test_old_samples_expire_after_gap():
    mean = PlanarMovingAverage(0.2)
    mean.add(pose(0, 0, 0, 1_000_000_000), 0.0)
    result = mean.add(pose(5, 6, 90, 2_000_000_000), 1.0)
    assert (result.pose.position.x, result.pose.position.y) == (5, 6)
    assert yaw_degrees(result) == pytest.approx(90)
