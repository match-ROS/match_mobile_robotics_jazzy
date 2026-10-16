"""A single MuR mapping session; global hardware TF is read, never overwritten."""
import os
import uuid
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from match_mur_navigation.settings import validate


def setup(context):
    robot, mode = (LaunchConfiguration(k).perform(context) for k in ('robot', 'mode'))
    params_path = LaunchConfiguration('params_file').perform(context)
    with open(params_path, encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    settings = validate(robot, mode, document['/**']['ros__parameters'])
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    namespace = f'/{robot}/mapping'
    params = [dict(settings, robot=robot, mode=mode, use_sim_time=sim,
                   session_id=LaunchConfiguration('session_id').perform(context))]
    session = Node(package='match_mur_navigation', executable='mapping_session',
                   namespace=namespace, parameters=params, output='screen')
    nodes = [session]
    actions = [session]
    private_tf = [('/tf', namespace + '/tf'), ('/tf_static', namespace + '/tf_static')]
    if mode == 'mocap':
        mapper = Node(package='match_mur_navigation', executable='mocap_mapper',
                      namespace=namespace, parameters=params, remappings=private_tf, output='screen')
        nodes.append(mapper)
        actions.append(mapper)
    else:
        defaults = os.path.join(get_package_share_directory('slam_toolbox'),
                                'config', 'mapper_params_online_async.yaml')
        # The installed Jazzy defaults are solver/tuning defaults; session values override them.
        with open(defaults, encoding='utf-8') as stream:
            slam = yaml.safe_load(stream)['slam_toolbox']['ros__parameters']
        slam.update(use_sim_time=sim, use_lifecycle_manager=False,
                    map_name=namespace + '/map_raw',
                    map_frame=f'{robot}/mapping/map', odom_frame=f'{robot}/mapping/odom',
                    base_frame=f'{robot}/base_footprint', scan_topic=namespace + '/scan',
                    resolution=settings['resolution'], min_laser_range=settings['min_range'],
                    max_laser_range=settings['max_range'],
                    map_update_interval=settings['map_update_interval'],
                    use_scan_matching=True, do_loop_closing=True, use_map_saver=False)
        mapper = LifecycleNode(package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', namespace=namespace, parameters=[slam],
            remappings=private_tf + [('pose', 'slam_pose')], output='screen')
        nodes.append(mapper)
        # Register before configuring so a fast transition cannot be missed.
        actions += [RegisterEventHandler(OnStateTransition(target_lifecycle_node=mapper,
            goal_state='inactive', entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(mapper),
                transition_id=Transition.TRANSITION_ACTIVATE))])), mapper,
            EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(mapper),
                                       transition_id=Transition.TRANSITION_CONFIGURE))]
    def backend_exit(_event, launch_context):
        if launch_context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(reason='Mapping backend exited'))]
    for node in nodes:
        actions.insert(0, RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=backend_exit)))
    if LaunchConfiguration('use_rviz').perform(context).lower() == 'true':
        actions.append(Node(package='match_mur_navigation', executable='open_mapping_rviz',
            arguments=['--robot', robot, '--mode', mode] + (['--sim'] if sim else []), output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', description='One of mur620a/b/c/d; required'),
        DeclareLaunchArgument('mode', default_value='mocap', choices=['mocap', 'slam']),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('session_id', default_value=uuid.uuid4().hex,
                              description='Session identity to reject stale GUI status and map messages'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('match_mur_navigation'), 'config', 'mapping.yaml')),
        OpaqueFunction(function=setup),
    ])
