"""Validated driving limits and Nav2 parameters for one MuR."""
import copy
import math
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from .settings import ROBOTS

DEFAULTS = dict(max_speed=0.25, max_reverse_speed=0.0, max_accel=0.25, max_decel=0.4,
    max_angular_speed=0.4, max_angular_accel=0.5, max_angular_decel=0.8,
    xy_goal_tolerance=0.1, yaw_goal_tolerance=0.15, inflation_radius=1.0,
    cost_scaling_factor=3.0, obstacle_range=8.0, raytrace_range=10.0,
    footprint_length=1.35, footprint_width=0.91, footprint_padding=0.05,
    progress_distance=0.1, progress_timeout=15.0, goal_timeout=300.0)

# UI bounds are also enforced for CLI/session files. All values use SI units.
BOUNDS = dict(max_speed=(0.03, 1.0), max_reverse_speed=(0., 0.3), max_accel=(0.05, 1.),
    max_decel=(0.05, 1.5), max_angular_speed=(0.05, 1.), max_angular_accel=(0.05, 2.),
    max_angular_decel=(0.05, 3.), xy_goal_tolerance=(0.02, 1.), yaw_goal_tolerance=(0.02, 0.8),
    inflation_radius=(0.1, 3.), cost_scaling_factor=(0.1, 20.), obstacle_range=(1., 20.),
    raytrace_range=(1., 30.), footprint_length=(1.35, 4.), footprint_width=(0.91, 4.),
    footprint_padding=(0.01, 0.5), progress_distance=(0.02, 0.5), progress_timeout=(3., 120.),
    goal_timeout=(10., 3600.))


def validate(settings):
    if not isinstance(settings, dict) or set(settings)-set(DEFAULTS):
        raise ValueError('Unbekannte Navigationseinstellungen')
    result = dict(DEFAULTS, **settings)
    for key, value in result.items():
        low, high = BOUNDS[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{key}: erwartet {low} bis {high}')
        result[key] = float(value)
    if result['max_reverse_speed'] > result['max_speed']:
        raise ValueError('Rückwärtsgeschwindigkeit darf die Vorwärtsgeschwindigkeit nicht überschreiten')
    if result['obstacle_range'] > result['raytrace_range']:
        raise ValueError('Raytracing-Reichweite muss mindestens der Hindernisreichweite entsprechen')
    minimum_inflation = math.hypot(result['footprint_width']/2+result['footprint_padding'],
                                   result['footprint_length']/2+result['footprint_padding'])
    if result['inflation_radius'] < minimum_inflation:
        raise ValueError(f'Inflationsradius muss den Footprint umschließen (mindestens {minimum_inflation:.3f} m)')
    if result['goal_timeout'] < result['progress_timeout']:
        raise ValueError('Zielzeitlimit muss mindestens dem Fortschrittszeitlimit entsprechen')
    return result


def parameters(robot, settings, use_sim_time=False, bt_file=''):
    if robot not in ROBOTS:
        raise ValueError('Unbekannter MuR')
    s = validate(settings)
    ns = f'/{robot}/navigation'
    base, odom = robot+'/base_footprint', robot+'/navigation/odom'
    source = Path(get_package_share_directory('nav2_bringup'))/'params/nav2_params.yaml'
    defaults = yaml.safe_load(source.read_text())
    controller = copy.deepcopy(defaults['controller_server']['ros__parameters'])
    controller.update(controller_frequency=20., odom_topic=ns+'/odom', enable_stamped_cmd_vel=False,
        failure_tolerance=0., progress_checker_plugins=['progress_checker'],
        goal_checker_plugins=['general_goal_checker'], controller_plugins=['FollowPath'])
    controller['progress_checker'].update(required_movement_radius=s['progress_distance'],
                                           movement_time_allowance=s['progress_timeout'])
    controller['general_goal_checker'].update(xy_goal_tolerance=s['xy_goal_tolerance'],
                                              yaw_goal_tolerance=s['yaw_goal_tolerance'])
    mppi = controller['FollowPath']
    mppi.update(plugin='nav2_mppi_controller::MPPIController', motion_model='DiffDrive',
        vx_max=s['max_speed'], vx_min=-s['max_reverse_speed'], vy_max=0.,
        wz_max=s['max_angular_speed'], ax_max=s['max_accel'], ax_min=-s['max_decel'],
        az_max=s['max_angular_accel'], visualize=False, batch_size=1500,
        model_dt=0.05, time_steps=80, transform_tolerance=0.2,
        vx_std=min(0.08,s['max_speed']/3), wz_std=min(0.15,s['max_angular_speed']/3),
        temperature=0.1)
    mppi['CostCritic'].update(consider_footprint=True)
    # The upstream example targets a faster robot with ~3 m/s² acceleration.
    # Match sampling and goal attraction to this slower, acceleration-limited base.
    mppi['GoalCritic'].update(cost_weight=15.0, threshold_to_consider=1.0)
    mppi['PathFollowCritic'].update(threshold_to_consider=0.4)
    x, y = s['footprint_length']/2, s['footprint_width']/2
    footprint = str([[x,y], [x,-y], [-x,-y], [-x,y]])
    def costmap(local):
        obstacles = dict(plugin='nav2_costmap_2d::ObstacleLayer', enabled=True,
                         observation_sources='front rear')
        for name in ('front', 'rear'):
            obstacles[name] = dict(topic=ns+'/'+name+'_scan', data_type='LaserScan',
                clearing=True, marking=True, inf_is_valid=True, expected_update_rate=0.,
                obstacle_max_range=s['obstacle_range'], raytrace_max_range=s['raytrace_range'],
                obstacle_min_range=0.1, raytrace_min_range=0.1, max_obstacle_height=2.0)
        result = dict(global_frame=odom if local else 'mocap', robot_base_frame=base,
            transform_tolerance=0.3, update_frequency=5. if local else 2., publish_frequency=2.,
            resolution=0.05, footprint=footprint, footprint_padding=s['footprint_padding'],
            track_unknown_space=True, always_send_full_costmap=True,
            plugins=['static_layer','obstacle_layer','inflation_layer'], obstacle_layer=obstacles,
            static_layer=dict(plugin='nav2_costmap_2d::StaticLayer', map_topic=ns+'/map',
                              map_subscribe_transient_local=True),
            inflation_layer=dict(plugin='nav2_costmap_2d::InflationLayer',
                inflation_radius=s['inflation_radius'], cost_scaling_factor=s['cost_scaling_factor']))
        if local:
            result.update(rolling_window=True, width=6, height=6)
        return result
    nodes = dict(controller_server=controller,
        planner_server=dict(expected_planner_frequency=2., planner_plugins=['GridBased'],
            GridBased=dict(plugin='nav2_smac_planner::SmacPlanner2D', tolerance=s['xy_goal_tolerance'],
                           allow_unknown=False, max_planning_time=2., cost_travel_multiplier=2.,
                           use_final_approach_orientation=False)),
        behavior_server=dict(local_frame=odom, global_frame='mocap', robot_base_frame=base,
            behavior_plugins=['wait'], wait=dict(plugin='nav2_behaviors::Wait'),
            local_costmap_topic='local_costmap/costmap_raw',
            global_costmap_topic='global_costmap/costmap_raw',
            local_footprint_topic='local_costmap/published_footprint',
            global_footprint_topic='global_costmap/published_footprint', enable_stamped_cmd_vel=False),
        bt_navigator=dict(global_frame='mocap', robot_base_frame=base, odom_topic=ns+'/odom',
            navigators=['navigate_to_pose'], navigate_to_pose=dict(plugin='nav2_bt_navigator::NavigateToPoseNavigator'),
            default_nav_to_pose_bt_xml=bt_file, bt_loop_duration=10, default_server_timeout=20,
            wait_for_service_timeout=1000, action_server_result_timeout=900.,
            error_code_names=['compute_path_error_code', 'follow_path_error_code']),
        velocity_smoother=dict(smoothing_frequency=20., feedback='OPEN_LOOP', scale_velocities=False,
            max_velocity=[s['max_speed'],0.,s['max_angular_speed']],
            min_velocity=[-s['max_reverse_speed'],0.,-s['max_angular_speed']],
            max_accel=[s['max_accel'],0.,s['max_angular_accel']],
            max_decel=[-s['max_decel'],0.,-s['max_angular_decel']],
            odom_topic=ns+'/odom', odom_duration=0.1, velocity_timeout=0.3,
            enable_stamped_cmd_vel=False, deadband_velocity=[0.,0.,0.]),
        collision_monitor=dict(base_frame_id=base, odom_frame_id=odom,
            cmd_vel_in_topic='cmd_vel_smoothed', cmd_vel_out_topic='cmd_vel_guarded',
            state_topic='collision_monitor_state', transform_tolerance=0.2, source_timeout=0.5,
            base_shift_correction=True, stop_pub_timeout=0.5, enable_stamped_cmd_vel=False,
            polygons=['FootprintApproach'], FootprintApproach=dict(type='polygon', action_type='approach',
                footprint_topic=ns+'/local_costmap/published_footprint', time_before_collision=1.5,
                simulation_time_step=0.1, min_points=3, visualize=False, enabled=True),
            observation_sources=['front','rear'],
            front=dict(type='scan', topic=ns+'/front_scan', enabled=True),
            rear=dict(type='scan', topic=ns+'/rear_scan', enabled=True)))
    nodes['local_costmap/local_costmap'] = costmap(True)
    nodes['global_costmap/global_costmap'] = costmap(False)
    return {ns+'/'+name: {'ros__parameters': dict(values, use_sim_time=use_sim_time)}
            for name, values in nodes.items()}
