"""Generic MUR base launch file.

This file is intentionally lightweight and parameterized so it can be included
by single-robot (mur620.launch.py) and multi-robot (multi_mur620.launch.py) launch
descriptions. It can also be launched directly for a quick single robot test.

Responsibilities (per include):
  * Optionally start Gazebo (gz sim) with a selected world (first include only)
  * Process mur_620 xacro and publish robot_description (scoped + global)
  * Spawn the entity at a given pose
  * Start ros2_control node (namespaced) and (optionally) lidar bridge
  * Bridge /clock only once (first include)

Arguments:
  robot_name (string)      Name / namespace / tf prefix (default 'mur620a')
  world (string)           World (without .world) for first include (default 'maze')
  x,y,z,Y (float)          Spawn pose (z default 0.07)
  use_sim_time (bool)      Use simulation time (default true)
  include_gz (bool)        Whether to start gz sim and /clock bridge (default true)
  lidar_bridge (bool)      Whether to bridge the robot's /scan topic (default true)
  start_controller_manager (bool) Start standalone ros2_control_node (default false)
  load_controllers (bool)  Spawn Gazebo ros2_control controllers (default true)
  laser_merger (bool)      Merge front/back scans to /<robot_name>/scan (default true)
  localization (bool)      Start map_server and AMCL (default false)
  navigation (bool)        Start Nav2 planner/controller/BT navigator (default false)
  ground_truth (bool)      Publish Gazebo model pose as ground truth topics (default true)
"""

import os
import tempfile
from copy import deepcopy

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_args():
    return [
        DeclareLaunchArgument('robot_name', default_value='mur620a'),
        DeclareLaunchArgument('world', default_value='maze'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.07'),
        DeclareLaunchArgument('Y', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('include_gz', default_value='true'),
        DeclareLaunchArgument('lidar_bridge', default_value='true'),
        DeclareLaunchArgument('start_controller_manager', default_value='false'),
        DeclareLaunchArgument('load_controllers', default_value='true'),
        DeclareLaunchArgument('laser_merger', default_value='true'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('navigation', default_value='false'),
        DeclareLaunchArgument('ground_truth', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_directory('mir_gazebo'),
                'maps',
                'maze.yaml',
            ),
        ),
    ]


def make_controller_config(robot_name, source_yaml):
    with open(source_yaml, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)

    mobile_params = config['mobile_base_controller']['ros__parameters']
    mobile_params['odom_frame_id'] = f'{robot_name}/odom'
    mobile_params['base_frame_id'] = f'{robot_name}/base_footprint'
    mobile_params['tf_frame_prefix_enable'] = False
    mobile_params['tf_frame_prefix'] = ''

    namespaced_config = deepcopy(config)
    namespaced_config[f'/{robot_name}/controller_manager'] = deepcopy(
        config['controller_manager']
    )

    controller_manager_params = config['controller_manager']['ros__parameters']
    for controller_name, controller_config in controller_manager_params.items():
        if controller_name == 'update_rate':
            continue

        controller_params = deepcopy(
            config.get(controller_name, {'ros__parameters': {}})
        )
        controller_params.setdefault('ros__parameters', {})
        controller_params['ros__parameters'].setdefault(
            'type', controller_config['type']
        )

        namespaced_config[controller_name] = deepcopy(controller_params)
        namespaced_config[f'/{robot_name}/{controller_name}'] = deepcopy(
            controller_params
        )

    out_dir = os.path.join(tempfile.gettempdir(), 'mur_launch_sim')
    os.makedirs(out_dir, exist_ok=True)
    safe_robot_name = robot_name.replace('/', '_')
    out_file = os.path.join(out_dir, f'{safe_robot_name}_mur_controllers.yaml')

    with open(out_file, 'w', encoding='utf-8') as config_file:
        yaml.safe_dump(namespaced_config, config_file, sort_keys=False)

    return out_file


def make_localization_config(robot_name, map_yaml, use_sim_time, x, y, yaw):
    config = {
        'map_server': {
            'ros__parameters': {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
                'frame_id': 'map',
                'topic_name': 'map',
            }
        },
        'amcl': {
            'ros__parameters': {
                'use_sim_time': use_sim_time,
                'alpha1': 0.2,
                'alpha2': 0.1,
                'alpha3': 0.1,
                'alpha4': 0.2,
                'alpha5': 0.2,
                'base_frame_id': f'{robot_name}/base_footprint',
                'global_frame_id': 'map',
                'map_topic': 'map',
                'map_subscribe_transient_local': True,
                'odom_frame_id': f'{robot_name}/odom',
                'scan_topic': f'/{robot_name}/scan',
                'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                'laser_model_type': 'likelihood_field',
                'laser_likelihood_max_dist': 2.0,
                'max_beams': 60,
                'max_particles': 5000,
                'min_particles': 500,
                'set_initial_pose': True,
                'initial_pose': {
                    'x': float(x),
                    'y': float(y),
                    'z': 0.0,
                    'yaw': float(yaw),
                },
                'tf_broadcast': True,
                'transform_tolerance': 0.2,
            }
        },
        'lifecycle_manager_localization': {
            'ros__parameters': {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }
        },
    }

    out_dir = os.path.join(tempfile.gettempdir(), 'mur_launch_sim')
    os.makedirs(out_dir, exist_ok=True)
    safe_robot_name = robot_name.replace('/', '_')
    out_file = os.path.join(out_dir, f'{safe_robot_name}_localization.yaml')

    with open(out_file, 'w', encoding='utf-8') as config_file:
        yaml.safe_dump(config, config_file, sort_keys=False)

    return out_file


def make_navigation_config(robot_name, use_sim_time):
    base_frame = f'{robot_name}/base_footprint'
    odom_frame = f'{robot_name}/odom'
    scan_topic = f'/{robot_name}/scan'
    odom_topic = f'/{robot_name}/mobile_base_controller/odom'

    footprint = '[[0.45, 0.36], [0.45, -0.36], [-0.45, -0.36], [-0.45, 0.36]]'
    config = {
        'bt_navigator': {
            'ros__parameters': {
                'use_sim_time': use_sim_time,
                'global_frame': 'map',
                'robot_base_frame': base_frame,
                'odom_topic': odom_topic,
                'bt_loop_duration': 10,
                'default_server_timeout': 20,
                'wait_for_service_timeout': 1000,
                'navigators': ['navigate_to_pose', 'navigate_through_poses'],
                'navigate_to_pose': {
                    'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator',
                },
                'navigate_through_poses': {
                    'plugin': 'nav2_bt_navigator::NavigateThroughPosesNavigator',
                },
                'error_code_names': [
                    'compute_path_error_code',
                    'follow_path_error_code',
                ],
            }
        },
        'bt_navigator_navigate_to_pose_rclcpp_node': {
            'ros__parameters': {
                'use_sim_time': use_sim_time,
            }
        },
        'bt_navigator_navigate_through_poses_rclcpp_node': {
            'ros__parameters': {
                'use_sim_time': use_sim_time,
            }
        },
        'controller_server': {
            'ros__parameters': {
                'use_sim_time': use_sim_time,
                'controller_frequency': 20.0,
                'costmap_update_timeout': 0.3,
                'enable_stamped_cmd_vel': True,
                'min_x_velocity_threshold': 0.001,
                'min_y_velocity_threshold': 0.001,
                'min_theta_velocity_threshold': 0.001,
                'failure_tolerance': 0.3,
                'progress_checker_plugins': ['progress_checker'],
                'goal_checker_plugins': ['general_goal_checker'],
                'controller_plugins': ['FollowPath'],
                'progress_checker': {
                    'plugin': 'nav2_controller::SimpleProgressChecker',
                    'required_movement_radius': 0.5,
                    'movement_time_allowance': 10.0,
                },
                'general_goal_checker': {
                    'plugin': 'nav2_controller::SimpleGoalChecker',
                    'stateful': True,
                    'xy_goal_tolerance': 0.25,
                    'yaw_goal_tolerance': 0.25,
                },
                'FollowPath': {
                    'plugin': 'dwb_core::DWBLocalPlanner',
                    'debug_trajectory_details': False,
                    'min_vel_x': 0.0,
                    'min_vel_y': 0.0,
                    'max_vel_x': 0.4,
                    'max_vel_y': 0.0,
                    'max_vel_theta': 1.0,
                    'min_speed_xy': 0.0,
                    'max_speed_xy': 0.4,
                    'min_speed_theta': 0.0,
                    'acc_lim_x': 1.0,
                    'acc_lim_y': 0.0,
                    'acc_lim_theta': 1.5,
                    'decel_lim_x': -1.0,
                    'decel_lim_y': 0.0,
                    'decel_lim_theta': -1.5,
                    'vx_samples': 20,
                    'vy_samples': 1,
                    'vtheta_samples': 20,
                    'sim_time': 1.7,
                    'linear_granularity': 0.05,
                    'angular_granularity': 0.025,
                    'transform_tolerance': 0.2,
                    'xy_goal_tolerance': 0.25,
                    'trans_stopped_velocity': 0.25,
                    'short_circuit_trajectory_evaluation': True,
                    'stateful': True,
                    'critics': [
                        'RotateToGoal',
                        'Oscillation',
                        'BaseObstacle',
                        'GoalAlign',
                        'PathAlign',
                        'PathDist',
                        'GoalDist',
                    ],
                    'BaseObstacle.scale': 0.02,
                    'PathAlign.scale': 32.0,
                    'PathAlign.forward_point_distance': 0.4,
                    'GoalAlign.scale': 24.0,
                    'GoalAlign.forward_point_distance': 0.4,
                    'PathDist.scale': 32.0,
                    'GoalDist.scale': 24.0,
                    'RotateToGoal.scale': 24.0,
                    'RotateToGoal.slowing_factor': 5.0,
                    'RotateToGoal.lookahead_time': -1.0,
                },
            }
        },
        'local_costmap': {
            'local_costmap': {
                'ros__parameters': {
                    'use_sim_time': use_sim_time,
                    'update_frequency': 5.0,
                    'publish_frequency': 2.0,
                    'global_frame': odom_frame,
                    'robot_base_frame': base_frame,
                    'rolling_window': True,
                    'width': 4,
                    'height': 4,
                    'resolution': 0.05,
                    'footprint': footprint,
                    'footprint_padding': 0.02,
                    'plugins': ['obstacle_layer', 'inflation_layer'],
                    'obstacle_layer': {
                        'plugin': 'nav2_costmap_2d::ObstacleLayer',
                        'enabled': True,
                        'observation_sources': 'scan',
                        'scan': {
                            'topic': scan_topic,
                            'max_obstacle_height': 2.0,
                            'clearing': True,
                            'marking': True,
                            'data_type': 'LaserScan',
                            'raytrace_max_range': 8.0,
                            'raytrace_min_range': 0.0,
                            'obstacle_max_range': 6.0,
                            'obstacle_min_range': 0.0,
                        },
                    },
                    'inflation_layer': {
                        'plugin': 'nav2_costmap_2d::InflationLayer',
                        'cost_scaling_factor': 3.0,
                        'inflation_radius': 0.65,
                    },
                    'always_send_full_costmap': True,
                }
            }
        },
        'global_costmap': {
            'global_costmap': {
                'ros__parameters': {
                    'use_sim_time': use_sim_time,
                    'update_frequency': 1.0,
                    'publish_frequency': 1.0,
                    'global_frame': 'map',
                    'robot_base_frame': base_frame,
                    'footprint': footprint,
                    'footprint_padding': 0.02,
                    'resolution': 0.05,
                    'track_unknown_space': True,
                    'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
                    'static_layer': {
                        'plugin': 'nav2_costmap_2d::StaticLayer',
                        'map_subscribe_transient_local': True,
                    },
                    'obstacle_layer': {
                        'plugin': 'nav2_costmap_2d::ObstacleLayer',
                        'enabled': True,
                        'observation_sources': 'scan',
                        'scan': {
                            'topic': scan_topic,
                            'max_obstacle_height': 2.0,
                            'clearing': True,
                            'marking': True,
                            'data_type': 'LaserScan',
                            'raytrace_max_range': 8.0,
                            'raytrace_min_range': 0.0,
                            'obstacle_max_range': 6.0,
                            'obstacle_min_range': 0.0,
                        },
                    },
                    'inflation_layer': {
                        'plugin': 'nav2_costmap_2d::InflationLayer',
                        'cost_scaling_factor': 3.0,
                        'inflation_radius': 0.65,
                    },
                    'always_send_full_costmap': True,
                }
            }
        },
        'planner_server': {
            'ros__parameters': {
                'use_sim_time': use_sim_time,
                'expected_planner_frequency': 20.0,
                'planner_plugins': ['GridBased'],
                'costmap_update_timeout': 1.0,
                'GridBased': {
                    'plugin': 'nav2_navfn_planner::NavfnPlanner',
                    'tolerance': 0.5,
                    'use_astar': False,
                    'allow_unknown': True,
                },
            }
        },
        'behavior_server': {
            'ros__parameters': {
                'use_sim_time': use_sim_time,
                'enable_stamped_cmd_vel': True,
                'local_costmap_topic': 'local_costmap/costmap_raw',
                'global_costmap_topic': 'global_costmap/costmap_raw',
                'local_footprint_topic': 'local_costmap/published_footprint',
                'global_footprint_topic': 'global_costmap/published_footprint',
                'cycle_frequency': 10.0,
                'behavior_plugins': ['spin', 'backup', 'drive_on_heading', 'wait'],
                'spin': {'plugin': 'nav2_behaviors::Spin'},
                'backup': {'plugin': 'nav2_behaviors::BackUp'},
                'drive_on_heading': {'plugin': 'nav2_behaviors::DriveOnHeading'},
                'wait': {'plugin': 'nav2_behaviors::Wait'},
                'local_frame': odom_frame,
                'global_frame': 'map',
                'robot_base_frame': base_frame,
                'transform_tolerance': 0.1,
                'simulate_ahead_time': 2.0,
                'max_rotational_vel': 1.0,
                'min_rotational_vel': 0.4,
                'rotational_acc_lim': 1.5,
            }
        },
        'lifecycle_manager_navigation': {
            'ros__parameters': {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }
        },
    }

    out_dir = os.path.join(tempfile.gettempdir(), 'mur_launch_sim')
    os.makedirs(out_dir, exist_ok=True)
    safe_robot_name = robot_name.replace('/', '_')
    out_file = os.path.join(out_dir, f'{safe_robot_name}_navigation.yaml')

    with open(out_file, 'w', encoding='utf-8') as config_file:
        yaml.safe_dump(config, config_file, sort_keys=False)

    return out_file


def controller_spawner(robot_name, controller_name, controllers_yaml):
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            controller_name,
            '--controller-manager', f'/{robot_name}/controller_manager',
            '--controller-manager-timeout', '60',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )


def launch_setup(context, *args, **kwargs):  # executed at runtime
    robot_name = LaunchConfiguration('robot_name').perform(context)
    world = LaunchConfiguration('world').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    Y = LaunchConfiguration('Y').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    include_gz = LaunchConfiguration('include_gz').perform(context) == 'true'
    lidar_bridge = LaunchConfiguration('lidar_bridge').perform(context) == 'true'
    start_controller_manager = (
        LaunchConfiguration('start_controller_manager').perform(context) == 'true'
    )
    load_controllers = LaunchConfiguration('load_controllers').perform(context) == 'true'
    laser_merger = LaunchConfiguration('laser_merger').perform(context) == 'true'
    localization = LaunchConfiguration('localization').perform(context) == 'true'
    navigation = LaunchConfiguration('navigation').perform(context) == 'true'
    ground_truth = LaunchConfiguration('ground_truth').perform(context) == 'true'
    map_yaml = LaunchConfiguration('map').perform(context)

    mur_description_path = get_package_share_directory('mur_description')
    mir_description_path = get_package_share_directory('mir_description')
    xacro_file = os.path.join(mur_description_path, 'urdf', 'mur_620.gazebo.xacro')
    base_controllers_yaml = os.path.join(mir_description_path, 'config', 'mur_controllers.yaml')
    controllers_yaml = make_controller_config(robot_name, base_controllers_yaml)
    doc = xacro.process_file(xacro_file, mappings={
        'use_sim': 'true',
        'tf_prefix': robot_name,
        'tf_prefix_mir': robot_name,
        'robot_namespace': robot_name,
        'simulation_controllers': controllers_yaml,
    })
    robot_desc = doc.toxml()

    nodes = []

    if include_gz:
        mir_gazebo_path = get_package_share_directory('mir_gazebo')
        nodes.append(
            SetEnvironmentVariable(
                name='GZ_SIM_RESOURCE_PATH',
                value=os.pathsep.join([
                    os.path.join(mir_gazebo_path, 'worlds'),
                    os.path.join(mir_gazebo_path, 'worlds', 'include'),
                ]),
            )
        )
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': f'{world}.world -v 4 -r'}.items(),
            )
        )
        # clock bridge only once
        nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='clock_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
            )
        )

    if ground_truth:
        gz_pose_topic = f'/world/{world}/pose/info'
        nodes.append(
            Node(
                package='mur_launch_sim',
                executable='gz_ground_truth_publisher',
                name=f'{robot_name}_ground_truth',
                parameters=[{
                    'gz_pose_topic': gz_pose_topic,
                    'robot_name': robot_name,
                    'output_frame_id': 'map',
                    'child_frame_id': f'{robot_name}/base_footprint',
                    'pose_topic': f'/{robot_name}/ground_truth/pose',
                    'odom_topic': f'/{robot_name}/ground_truth/odom',
                    'use_sim_time': use_sim_time,
                }],
                output='screen',
            ),
        )

    # state publisher (namespaced)
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_rsp',
            namespace=robot_name,
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': use_sim_time},
                {'frame_prefix': f'{robot_name}/'},
            ],
            output='screen',
        )
    )

    # global (for gz_ros2_control consumption if required)
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_global_rsp',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': use_sim_time},
                {'publish_frequency': 0.0},  # suppress periodic republishes (supported in newer versions)
            ],
            remappings=[('/tf', 'unused_tf'), ('/tf_static', 'unused_tf_static')],  # optional to avoid duplicates
            output='screen',
        )
    )

    if start_controller_manager:
        nodes.append(
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                namespace=robot_name,
                name='controller_manager',
                parameters=[
                    controllers_yaml,
                    {'robot_description': robot_desc},
                    {'use_sim_time': use_sim_time},
                ],
                output='screen',
            )
        )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'{robot_name}_spawn',
        arguments=[
            '-string', robot_desc,
            '-name', robot_name,
            '-x', x, '-y', y, '-z', z,
            '-Y', Y,
        ],
        output='screen',
    )
    nodes.append(spawn_entity)

    if load_controllers:
        nodes.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[
                        controller_spawner(
                            robot_name, 'joint_state_broadcaster', controllers_yaml
                        ),
                        controller_spawner(
                            robot_name, 'mobile_base_controller', controllers_yaml
                        ),
                        controller_spawner(
                            robot_name, 'lift_controller_l', controllers_yaml
                        ),
                        controller_spawner(
                            robot_name, 'lift_controller_r', controllers_yaml
                        ),
                    ],
                )
            )
        )

    if lidar_bridge:
        nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{robot_name}_lidar_bridge',
                arguments=[
                    f'/{robot_name}/f_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    f'/{robot_name}/b_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                ],
                remappings=[
                    (f'/{robot_name}/f_scan', f'/{robot_name}/f_scan_raw'),
                    (f'/{robot_name}/b_scan', f'/{robot_name}/b_scan_raw'),
                ],
                output='screen',
            )
        )
        nodes.extend([
            Node(
                package='mur_launch_sim',
                executable='laserscan_frame_republisher.py',
                name=f'{robot_name}_front_scan_frame',
                parameters=[{
                    'input_topic': f'/{robot_name}/f_scan_raw',
                    'output_topic': f'/{robot_name}/f_scan',
                    'frame_id': f'{robot_name}/front_laser_link',
                    'use_sim_time': use_sim_time,
                }],
                output='screen',
            ),
            Node(
                package='mur_launch_sim',
                executable='laserscan_frame_republisher.py',
                name=f'{robot_name}_back_scan_frame',
                parameters=[{
                    'input_topic': f'/{robot_name}/b_scan_raw',
                    'output_topic': f'/{robot_name}/b_scan',
                    'frame_id': f'{robot_name}/back_laser_link',
                    'use_sim_time': use_sim_time,
                }],
                output='screen',
            ),
        ])

    if laser_merger:
        nodes.append(
            Node(
                package='ira_laser_tools',
                executable='laserscan_multi_merger',
                name=f'{robot_name}_laser_scan_merger',
                parameters=[
                    {
                        'laserscan_topics': f'/{robot_name}/b_scan /{robot_name}/f_scan',
                        'destination_frame': f'{robot_name}/base_footprint',
                        'scan_destination_topic': f'/{robot_name}/scan_merged_raw',
                        'cloud_destination_topic': f'/{robot_name}/scan_cloud',
                        'min_height': -0.25,
                        'max_completion_time': 0.05,
                        'max_merge_time_diff': 0.005,
                        'use_sim_time': use_sim_time,
                        'best_effort': False,
                    }
                ],
                output='screen',
            )
        )
        nodes.append(
            Node(
                package='mur_launch_sim',
                executable='laserscan_frame_republisher.py',
                name=f'{robot_name}_merged_scan_frame',
                parameters=[{
                    'input_topic': f'/{robot_name}/scan_merged_raw',
                    'output_topic': f'/{robot_name}/scan',
                    'frame_id': f'{robot_name}/base_footprint',
                    'input_reliability': 'best_effort',
                    'output_reliability': 'reliable',
                    'use_sim_time': use_sim_time,
                }],
                output='screen',
            )
        )

    if localization:
        localization_yaml = make_localization_config(
            robot_name, map_yaml, use_sim_time, x, y, Y
        )
        nodes.extend([
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[localization_yaml],
                output='screen',
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                parameters=[localization_yaml],
                output='screen',
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                parameters=[localization_yaml],
                output='screen',
            ),
        ])

    if navigation:
        navigation_yaml = make_navigation_config(robot_name, use_sim_time)
        cmd_vel_topic = f'/{robot_name}/mobile_base_controller/cmd_vel'
        remappings = [
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
        nodes.extend([
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                parameters=[navigation_yaml],
                remappings=remappings + [('cmd_vel', cmd_vel_topic)],
                output='screen',
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                parameters=[navigation_yaml],
                remappings=remappings,
                output='screen',
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                parameters=[navigation_yaml],
                remappings=remappings + [('cmd_vel', cmd_vel_topic)],
                output='screen',
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                parameters=[navigation_yaml],
                remappings=remappings,
                output='screen',
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                parameters=[navigation_yaml],
                output='screen',
            ),
        ])

    return nodes


def generate_launch_description():
    ld = LaunchDescription(declare_args())
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
