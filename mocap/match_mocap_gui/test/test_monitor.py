"""Mocap status handling while localization is held."""

import time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from match_mocap_gui.mocap_gui_module import MocapMonitor


def test_held_map_pose_stays_live_when_raw_body_is_occluded():
    monitor = MocapMonitor()
    snapshots = []
    monitor.snapshot.connect(snapshots.append)
    raw = PoseStamped()
    raw.header.frame_id = 'mocap'
    mapped = PoseStamped()
    mapped.header.frame_id = 'map'
    monitor._on_raw('mur620a', raw)
    monitor._on_map('mur620a', mapped)
    monitor._on_frozen('mur620a', Bool(data=True))
    monitor._emit_snapshot()
    assert snapshots[-1]['mur620a']['live']
    assert snapshots[-1]['mur620a']['map_live']
    assert snapshots[-1]['mur620a']['frozen'] is True

    monitor._times['mur620a'].clear()
    monitor._on_map('mur620a', mapped)
    monitor._emit_snapshot()
    assert not snapshots[-1]['mur620a']['live']
    assert snapshots[-1]['mur620a']['map_live']
    assert snapshots[-1]['mur620a']['frozen'] is True

    monitor._last_frozen['mur620a'] = (time.monotonic() - 3.0, True)
    monitor._emit_snapshot()
    assert snapshots[-1]['mur620a']['frozen'] is None
