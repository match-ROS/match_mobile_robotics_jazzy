"""Saved map and optional AMCL experiment, with a private TF tree only."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from match_mur_navigation.saved_map import default_map, map_metadata
from match_mur_navigation.settings import ROBOTS


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    robot = value('robot')
    if robot not in ROBOTS:
        raise ValueError('Unknown MuR')
    path, _ = map_metadata(value('map'))
    namespace = f'/{robot}/localization'
    sim = value('use_sim_time').lower() == 'true'
    private_tf = [('/tf', namespace+'/tf'), ('/tf_static', namespace+'/tf_static')]
    nodes = []
    managed = []
    if value('load_map').lower() == 'true':
        nodes.append(Node(package='nav2_map_server', executable='map_server', name='map_server',
                  namespace=namespace, parameters=[dict(yaml_filename=str(path), frame_id='mocap',
                                                        use_sim_time=sim)], output='screen'))
        managed.append('map_server')
    if value('localize').lower() == 'true':
        nodes += [Node(package='match_mur_navigation', executable='localization_session',
            namespace=namespace, parameters=[dict(robot=robot, use_sim_time=sim,
                output_directory=value('output_directory'), map_yaml=str(path),
                sensor_namespace=value('sensor_namespace'),
                stationary_updates=value('stationary_updates').lower() == 'true')], output='screen'),
            Node(package='nav2_amcl', executable='amcl', name='amcl', namespace=namespace,
                remappings=private_tf, parameters=[dict(use_sim_time=sim,
                    base_frame_id=robot+'/base_footprint', odom_frame_id=robot+'/localization/odom',
                    global_frame_id='mocap', scan_topic='scan', map_topic='map',
                    robot_model_type='nav2_amcl::DifferentialMotionModel',
                    laser_model_type='likelihood_field', laser_max_range=10.0,
                    min_particles=500, max_particles=2000, max_beams=90,
                    update_min_d=0.02, update_min_a=0.02, transform_tolerance=0.2,
                    tf_broadcast=True, set_initial_pose=False)], output='screen')]
        managed.append('amcl')
    if not managed:
        raise ValueError('Enable load_map or localize')
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='amcl_manager' if value('load_map').lower() == 'false' else 'localization_manager', namespace=namespace,
        parameters=[dict(use_sim_time=sim, autostart=True, node_names=managed)], output='screen'))
    def backend_exit(_event, launch_context):
        return [] if launch_context.is_shutdown else [EmitEvent(event=Shutdown(reason='Localization backend exited'))]
    return [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=backend_exit))
            for node in nodes] + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='mur620c'),
        DeclareLaunchArgument('map', default_value=str(default_map())),
        DeclareLaunchArgument('localize', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('load_map', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('stationary_updates', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('sensor_namespace', default_value='',
            description='Optional separate sensor/TF input namespace for read-only experiments'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('output_directory', default_value='/tmp/mur_localization'),
        OpaqueFunction(function=setup)])
