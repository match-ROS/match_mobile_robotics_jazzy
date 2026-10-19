"""Legacy cleanup cannot stop hardware, shells or another robot's mapping."""

import signal
from unittest.mock import Mock

import pytest

from match_mur_navigation import stop_session


@pytest.mark.parametrize('arguments', [
    ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'launch', 'match_mur_navigation',
     'mapping.launch.py', 'robot:=mur620c', 'mode:=mocap'],
    ['/ws/install/match_mur_navigation/lib/match_mur_navigation/mapping_session',
     '--ros-args', '-r', '__ns:=/mur620c/mapping'],
    ['/ws/install/match_mur_navigation/lib/match_mur_navigation/mocap_mapper',
     '--ros-args', '-r', '__ns:=/mur620c/mapping'],
    ['/usr/bin/python3', '-m', 'match_mur_navigation.session', '--ros-args',
     '-r', '__ns:=/mur620c/mapping'],
])
def test_exact_mapping_launch_or_backend_is_recognized(arguments):
    assert stop_session.is_mapping_process(arguments, 'mur620c')
    assert not stop_session.is_mapping_process(arguments, 'mur620a')


@pytest.mark.parametrize('arguments', [
    ['bash', '-c', 'ros2 launch match_mur_navigation mapping.launch.py robot:=mur620c'],
    ['ros2', 'launch', 'mur_launch_hardware', 'mur_620.launch.py', 'robot_name:=mur620c'],
    ['ros2', 'launch', 'match_mur_navigation', 'mapping.launch.py', 'robot:=mur620a'],
    ['ros2', 'launch', 'match_mur_navigation', 'mapping.launch.py',
     'robot:=mur620c', 'robot:=mur620a'],
    ['/ws/mir_bridge', '--ros-args', '-r', '__ns:=/mur620c'],
    ['/ws/mocap_mapper', '--ros-args', '-r', '__ns:=/mur620c/mapping_other'],
    ['python3', '-m', 'match_mocap_ros2.qualisys_ssh_bridge'],
])
def test_unrelated_processes_are_never_selected(arguments):
    assert not stop_session.is_mapping_process(arguments, 'mur620c')


def test_stop_sends_only_sigint_and_waits_for_exact_processes(monkeypatch):
    states = iter([[(123, 'start')], [(123, 'start')], []])
    monkeypatch.setattr(stop_session, 'local_processes', lambda robot: next(states))
    kill = Mock()
    monkeypatch.setattr(stop_session.os, 'kill', kill)
    stop_session.stop_local_session('mur620c')
    kill.assert_called_once_with(123, signal.SIGINT)


def test_unresponsive_backend_is_not_force_killed(monkeypatch):
    monkeypatch.setattr(stop_session, 'local_processes', lambda robot: [(123, 'start')])
    clock = iter([0.0, 10.0])
    monkeypatch.setattr(stop_session.time, 'monotonic', lambda: next(clock))
    kill = Mock()
    monkeypatch.setattr(stop_session.os, 'kill', kill)
    with pytest.raises(RuntimeError, match='kein erzwungenes'):
        stop_session.stop_local_session('mur620c', timeout=1.0)
    kill.assert_called_once_with(123, signal.SIGINT)


def test_reused_pid_is_not_signalled(monkeypatch):
    states = iter([[(123, 'old')], [(123, 'new')], []])
    monkeypatch.setattr(stop_session, 'local_processes', lambda robot: next(states))
    kill = Mock()
    monkeypatch.setattr(stop_session.os, 'kill', kill)
    stop_session.stop_local_session('mur620c')
    kill.assert_not_called()
