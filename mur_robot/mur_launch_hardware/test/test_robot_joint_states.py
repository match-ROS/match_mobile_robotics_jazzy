"""Keep identically named joints on different MuRs out of each other's model."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from launch import LaunchContext
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


spec = importlib.util.spec_from_file_location(
    'mur_hardware_launch', Path(__file__).parents[1] / 'launch/mur_620.launch.py')
hardware = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hardware)


def launch_context(robot, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(robot_name=robot, robot_profile=robot, **overrides)
    for declaration in hardware.declare_arguments():
        declaration.execute(context)
    return context


@pytest.mark.parametrize('robot', ['mur620a', 'mur620b', 'mur620c', 'mur620d'])
def test_hardware_state_producers_and_consumers_use_the_same_robot_topic(robot, monkeypatch, tmp_path):
    context = launch_context(robot)
    topic = f'/{robot}/joint_states'
    records = []
    real_node = hardware.Node
    # Never overwrite parameter files belonging to a running hardware launch.
    monkeypatch.setattr(hardware, 'tempfile', SimpleNamespace(gettempdir=lambda: str(tmp_path)))

    def record_node(**kwargs):
        records.append(kwargs)
        return real_node(**kwargs)

    monkeypatch.setattr(hardware, 'Node', record_node)
    actions = hardware.launch_setup(context)
    for side in ('l', 'r'):
        hardware.controller_spawner(f'{robot}/UR10_{side}', ['joint_state_broadcaster'])

    checked = set()
    for node in records:
        executable = node['executable']
        for params in node.get('parameters', []):
            if isinstance(params, dict) and 'joint_states_topic' in params:
                assert params['joint_states_topic'] == topic
                checked.add(executable)
        args = node.get('arguments', [])
        if '--joint-states-topic' in args:
            assert args[args.index('--joint-states-topic')+1] == topic
            checked.add(executable)
        if executable == 'spawner':
            assert f'--controller-ros-args=-r joint_states:={topic}' in args
            checked.add(executable)
        if node.get('name') == f'{robot}_rsp':
            assert ('joint_states', topic) in node['remappings']
            checked.add('mur_rsp')

    assert checked == {
        'ewellix_dual_state_to_joint_state.py', 'fake_mir_wheel_joint_states.py',
        'arm_velocity_safety_node', 'moveit_trajectory_controller_proxy.py',
        'jparse_velocity_controller', 'jparse_move_action_server.py', 'spawner', 'mur_rsp',
    }
    # MoveIt is included as a launch description instead of constructed as a Node.
    moveit = next(action for action in actions
                  if hasattr(action, 'launch_arguments') and
                  any(name == 'joint_states_topic' for name, _ in action.launch_arguments))
    moveit_args = {
        perform_substitutions(context, normalize_to_list_of_substitutions(name)):
        perform_substitutions(context, normalize_to_list_of_substitutions(value))
        for name, value in moveit.launch_arguments
    }
    assert moveit_args['joint_states_topic'] == topic
    for side in ('l', 'r'):
        controller_file = tmp_path / 'mur_launch_hardware' / f'{robot}_UR10_{side}_ur_controllers.yaml'
        config = yaml.safe_load(controller_file.read_text())
        params = config[f'/{robot}/UR10_{side}/integrated_cartesian_admittance_controller']['ros__parameters']
        assert params['collision_joint_states_topic'] == topic


def test_collision_topic_can_still_be_overridden():
    context = launch_context('mur620a',
                             integrated_controller_collision_joint_states_topic='/custom/states')
    assert context.launch_configurations['integrated_controller_collision_joint_states_topic'] == '/custom/states'


@pytest.mark.parametrize('robot', ['mur620a', 'mur620b', 'mur620c', 'mur620d'])
def test_base_localization_defaults_to_auto_with_robot_specific_mocap_input(robot):
    context = launch_context(robot)
    action = hardware.make_mir_hardware_launch(robot, False)
    arguments = {
        perform_substitutions(context, normalize_to_list_of_substitutions(name)):
        perform_substitutions(context, normalize_to_list_of_substitutions(value))
        for name, value in action.launch_arguments
    }
    assert arguments['localization_type'] == 'auto'
    assert arguments['publish_localization_tf'] == 'true'
    assert arguments['localization_topic'] == f'/qualisys_map/{robot}/pose'
    assert arguments['localization_timeout'] == '1.0'
    assert arguments['tf_excluded_children'] == 'odom base_footprint base_link'


def test_spawner_locks_are_shared_within_one_arm_and_separate_across_arms(monkeypatch):
    records = []
    real_node = hardware.Node
    def record_node(**kwargs):
        records.append(kwargs)
        return real_node(**kwargs)
    monkeypatch.setattr(hardware, 'Node', record_node)
    for robot in ('mur620a', 'mur620d'):
        for side in ('l', 'r'):
            hardware.controller_spawner(f'{robot}/UR10_{side}', ['joint_state_broadcaster'])
            hardware.controller_spawner(f'{robot}/UR10_{side}', ['forward_velocity_controller'], active=False)
    homes = [node['additional_env']['ROS_HOME'] for node in records]
    assert len(set(homes)) == 4
    assert all(homes[i] == homes[i+1] for i in range(0, 8, 2))


def test_state_broadcasters_precede_motion_spawner_and_startup_requires_success(monkeypatch):
    context = launch_context('mur620a')
    nodes, handlers = [], []
    real_node, real_on_exit = hardware.Node, hardware.OnProcessExit
    def record_node(**kwargs):
        node = real_node(**kwargs)
        nodes.append((node, kwargs))
        return node
    def record_exit(**kwargs):
        handlers.append(kwargs)
        return real_on_exit(**kwargs)
    monkeypatch.setattr(hardware, 'Node', record_node)
    monkeypatch.setattr(hardware, 'OnProcessExit', record_exit)
    logger = Mock()
    monkeypatch.setattr(hardware, 'get_logger', lambda _: logger)
    driver = hardware.make_ur_driver('l', 'mur620a', '/tmp/controllers.yaml',
                                     '/tmp/update_rate.yaml', '/tmp/kinematics.yaml')
    opaque = next(action for action in driver.get_sub_entities() if isinstance(action, OpaqueFunction))
    actions = opaque.execute(context)
    spawners = [(node, kwargs) for node, kwargs in nodes if kwargs['executable'] == 'spawner']
    active, inactive = (entry[0] for entry in spawners)
    assert 'joint_state_broadcaster' in spawners[0][1]['arguments']
    assert '--inactive' not in spawners[0][1]['arguments']
    assert '--inactive' in spawners[1][1]['arguments']
    assert actions[-1] is active
    assert inactive not in actions
    assert all(isinstance(action, RegisterEventHandler) for action in actions[:-1])
    assert handlers[0]['target_action'] is active
    assert handlers[0]['on_exit'](SimpleNamespace(returncode=0), context) == [inactive]
    assert handlers[1]['target_action'] is inactive
    assert len(handlers[1]['on_exit'](SimpleNamespace(returncode=0), context)) == 1
    for handler in handlers:
        for code in (1, -15):
            followup = handler['on_exit'](SimpleNamespace(returncode=code), context)
            assert followup == []
    assert logger.error.call_count == 4
