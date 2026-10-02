"""MoveIt must preserve whether Cartesian motion was explicitly enabled."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location(
    'moveit_proxy', Path(__file__).parents[1] / 'scripts/moveit_trajectory_controller_proxy.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
Proxy = module.MoveItTrajectoryControllerProxy


@pytest.mark.parametrize('active', [False, True])
def test_restore_only_previously_active_cartesian_controller(active):
    switches = []
    node = SimpleNamespace(
        velocity_controller='cartesian', trajectory_controller='trajectory',
        _controller_states=lambda: {'cartesian': 'active' if active else 'inactive'},
        _publish_zero_velocity=lambda: None,
        _switch_controllers=lambda **kw: switches.append(kw) or True,
        args=SimpleNamespace(post_result_settle_sec=0.0),
    )
    assert Proxy._switch_to_trajectory(node)
    assert Proxy._switch_to_velocity(node)
    assert len(switches) == (2 if active else 1)


def test_missing_controller_state_refuses_goal():
    node = SimpleNamespace(_controller_states=lambda: {})
    assert not Proxy._switch_to_trajectory(node)
