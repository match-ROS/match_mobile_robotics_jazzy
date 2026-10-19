"""Nav2 planning/control with private TF and a supervised hardware output."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from match_mur_navigation.navigation_config import parameters, validate
from match_mur_navigation.saved_map import default_map, map_metadata
from match_mur_navigation.settings import ROBOTS


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    robot = value('robot')
    if robot not in ROBOTS:
        raise ValueError('Unknown MuR')
    settings = validate(yaml.safe_load(Path(value('params_file')).read_text()))
    map_path, _ = map_metadata(value('map'))
    share = Path(get_package_share_directory('match_mur_navigation'))
    directory = Path(value('output_directory') or tempfile.mkdtemp(prefix='mur_nav2_'))
    directory.mkdir(parents=True, exist_ok=True)
    sim = value('use_sim_time') == 'true'
    namespace = '/'+robot+'/navigation'
    nav2_params = directory/'nav2_parameters.yaml'
    nav2_params.write_text(yaml.safe_dump(parameters(robot, settings, sim, str(share/'config/navigate.xml'))))
    private_tf = [('/tf',namespace+'/tf'), ('/tf_static',namespace+'/tf_static')]
    nodes = []
    managed = []
    for package, executable, extra in (
        ('nav2_controller','controller_server',[('cmd_vel','cmd_vel_nav')]),
        ('nav2_planner','planner_server',[]),
        ('nav2_behaviors','behavior_server',[('cmd_vel','cmd_vel_nav')]),
        ('nav2_bt_navigator','bt_navigator',[('navigate_to_pose','nav2_navigate_to_pose')]),
        ('nav2_velocity_smoother','velocity_smoother',[('cmd_vel','cmd_vel_nav')]),
        ('nav2_collision_monitor','collision_monitor',[])):
        managed.append(executable)
        nodes.append(Node(package=package, executable=executable, name=executable, namespace=namespace,
                          parameters=[str(nav2_params)], remappings=private_tf+extra, output='screen'))
    nodes += [Node(package='nav2_map_server', executable='map_server', name='map_server', namespace=namespace,
        parameters=[dict(yaml_filename=str(map_path), frame_id='mocap', use_sim_time=sim)], output='screen'),
        Node(package='match_mur_navigation', executable='navigation_session', namespace=namespace,
            parameters=[dict(settings, robot=robot, map_yaml=str(map_path), localization_source=value('localization_source'),
                use_sim_time=sim, sensor_namespace=value('sensor_namespace'),
                output_topic=value('output_topic') or '/'+robot+'/cmd_vel_stamped')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='navigation_manager',
            namespace=namespace, parameters=[dict(use_sim_time=sim, autostart=True,
                                                  node_names=['map_server']+managed)], output='screen')]
    def exited(_event, launch_context):
        return [] if launch_context.is_shutdown else [EmitEvent(event=Shutdown(reason='Navigation backend exited'))]
    return [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=exited)) for node in nodes]+nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot'),
        DeclareLaunchArgument('map', default_value=str(default_map())),
        DeclareLaunchArgument('localization_source', default_value='mocap', choices=['mocap','amcl']),
        DeclareLaunchArgument('params_file', default_value=str(Path(get_package_share_directory(
            'match_mur_navigation'))/'config/navigation.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('output_directory', default_value=''),
        DeclareLaunchArgument('sensor_namespace', default_value='', description='Isolated sensor inputs for testing'),
        DeclareLaunchArgument('output_topic', default_value='', description='Override motor output for simulation/testing'),
        OpaqueFunction(function=setup)])
