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


@pytest.mark.parametrize('failure', ['disconnect', 'timeout', 'cancel'])
def test_execution_failure_cancels_and_disables_controllers(failure, monkeypatch):
    import threading
    from unittest.mock import Mock
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module, 'execution_timeout', lambda _: -1 if failure == 'timeout' else 10)
    pending = SimpleNamespace(done=lambda: False)
    cancel_response = SimpleNamespace(goals_canceling=[object()])
    real_goal = Mock(accepted=True)
    real_goal.get_result_async.return_value = pending
    real_goal.cancel_goal_async.return_value = SimpleNamespace(result=lambda: cancel_response)
    sent = SimpleNamespace(result=lambda: real_goal)
    readiness = iter([True, True, failure != 'disconnect'])
    node = SimpleNamespace(
        args=SimpleNamespace(action_timeout=1.0, check_ur_program=True),
        _driver_program_running=None,
        _client_goal_lock=threading.Lock(), _active_client_goal=None,
        _restore_velocity_after_goal=True,
        trajectory_controller='trajectory', velocity_controller='cartesian',
        get_logger=lambda: Mock(), _trajectory_summary=lambda _: '',
        _program_ready=lambda: next(readiness),
        _switch_to_trajectory=lambda: True,
        _goal_with_execution_tolerances=lambda request: request,
        _wait_for_future=lambda *args: True,
        trajectory_client=SimpleNamespace(wait_for_server=lambda **kw: True,
                                         send_goal_async=lambda *args, **kw: sent),
        _switch_to_velocity=Mock(), _switch_controllers=Mock(return_value=True),
    )
    node._stop_execution = lambda: Proxy._stop_execution(node)
    trajectory = JointTrajectory(points=[JointTrajectoryPoint()])
    goal = Mock(request=SimpleNamespace(trajectory=trajectory), is_cancel_requested=False)
    # Cancellation arrives only after the real controller accepted the goal.
    def get_result():
        goal.is_cancel_requested = failure == 'cancel'
        return pending
    real_goal.get_result_async.side_effect = get_result
    result = Proxy._execute_goal(node, goal)
    assert result.error_code != module.FollowJointTrajectory.Result.SUCCESSFUL
    real_goal.cancel_goal_async.assert_called()
    node._switch_to_velocity.assert_not_called()
    assert node._switch_controllers.call_args.kwargs['activate'] == []
    assert node._switch_controllers.call_args.kwargs['deactivate'] == ['trajectory', 'cartesian']
    assert not node._restore_velocity_after_goal
    assert node._active_client_goal is None
    if failure == 'cancel':
        goal.canceled.assert_called_once()
    else:
        goal.abort.assert_called_once()


def test_not_ready_never_activates_controller():
    from unittest.mock import Mock
    node = SimpleNamespace(get_logger=lambda: Mock(), _trajectory_summary=lambda _: '',
                           _program_ready=lambda: False, _switch_to_trajectory=Mock())
    goal = Mock(is_cancel_requested=False)
    Proxy._execute_goal(node, goal)
    node._switch_to_trajectory.assert_not_called()
    goal.abort.assert_called_once()
