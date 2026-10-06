"""Keep identically named joints on different MuRs out of each other's model."""

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
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
def test_hardware_state_producers_and_consumers_use_the_same_robot_topic(robot, monkeypatch):
    context = launch_context(robot)
    topic = f'/{robot}/joint_states'
    records = []
    real_node = hardware.Node

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
        controller_file = Path('/tmp/mur_launch_hardware') / f'{robot}_UR10_{side}_ur_controllers.yaml'
        config = yaml.safe_load(controller_file.read_text())
        params = config[f'/{robot}/UR10_{side}/integrated_cartesian_admittance_controller']['ros__parameters']
        assert params['collision_joint_states_topic'] == topic


def test_collision_topic_can_still_be_overridden():
    context = launch_context('mur620a',
                             integrated_controller_collision_joint_states_topic='/custom/states')
    assert context.launch_configurations['integrated_controller_collision_joint_states_topic'] == '/custom/states'
