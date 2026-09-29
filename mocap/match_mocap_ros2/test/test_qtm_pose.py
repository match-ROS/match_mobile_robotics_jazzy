import math

import pytest

from match_mocap_ros2.qualisys_ssh_bridge import pose_from_qtm


def test_qtm_column_major_rotation_and_millimetres():
    body = {
        'position_mm': [1200.0, -250.0, 40.0],
        'rotation_column_major': [0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    }
    pose = pose_from_qtm(body, None, 'mocap')
    assert pose.header.frame_id == 'mocap'
    assert pose.pose.position.x == pytest.approx(1.2)
    assert pose.pose.position.y == pytest.approx(-0.25)
    assert pose.pose.position.z == pytest.approx(0.04)
    assert pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))


def test_qtm_untracked_pose_is_rejected():
    body = {
        'position_mm': [float('nan'), 0.0, 0.0],
        'rotation_column_major': [1, 0, 0, 0, 1, 0, 0, 0, 1],
    }
    with pytest.raises(ValueError):
        pose_from_qtm(body, None, 'mocap')
