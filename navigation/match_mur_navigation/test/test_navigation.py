import math
import json
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformException

from match_mur_navigation.navigation_config import DEFAULTS, parameters, validate
from match_mur_navigation.navigation_session import NavigationSession, map_to_odom
from match_mur_navigation.rviz import configuration


def test_limits_reach_planner_controller_costmaps_and_smoother():
    values = validate(dict(max_speed=.33,max_accel=.22,max_decel=.45,max_angular_speed=.55,
                          xy_goal_tolerance=.12,yaw_goal_tolerance=.2,inflation_radius=.9))
    p = parameters('mur620c',values,bt_file='/tmp/navigation.xml')
    get = lambda name: p['/mur620c/navigation/'+name]['ros__parameters']
    c = get('controller_server')
    assert c['FollowPath']['vx_max'] == .33
    assert c['FollowPath']['ax_max'] == .22
    assert c['FollowPath']['ax_min'] == -.45
    assert c['FollowPath']['wz_max'] == .55
    assert c['FollowPath']['CostCritic']['consider_footprint']
    assert c['general_goal_checker']['xy_goal_tolerance'] == .12
    assert c['general_goal_checker']['yaw_goal_tolerance'] == .2
    assert get('velocity_smoother')['max_velocity'] == [.33,0.,.55]
    assert get('velocity_smoother')['max_accel'][0] == .22
    assert get('velocity_smoother')['max_decel'][0] == -.45
    assert get('planner_server')['GridBased']['plugin'] == 'nav2_smac_planner::SmacPlanner2D'
    assert not get('planner_server')['GridBased']['allow_unknown']
    for name in ('global_costmap/global_costmap','local_costmap/local_costmap'):
        costmap = get(name)
        assert costmap['inflation_layer']['inflation_radius'] == .9
        assert costmap['obstacle_layer']['observation_sources'] == 'front rear'
        assert costmap['footprint_padding'] == .05
    assert get('bt_navigator')['default_nav_to_pose_bt_xml'] == '/tmp/navigation.xml'


@pytest.mark.parametrize('settings',[{'max_speed':float('nan')},{'max_speed':True},
    {'max_speed':2.},{'max_reverse_speed':.3,'max_speed':.2}, {'obstacle_range':11.},
    {'footprint_width':.5},{'inflation_radius':.2},{'unknown':1}])
def test_invalid_driving_limits_are_rejected(settings):
    with pytest.raises(ValueError):
        validate(settings)


def test_saved_map_goals_are_transformed_not_relabelled():
    buffer = Buffer()
    t = TransformStamped(child_frame_id='mocap')
    t.header.frame_id = 'map'
    t.transform.translation.x,t.transform.translation.y = 10.,20.
    t.transform.rotation.z,t.transform.rotation.w = math.sin(math.pi/4),math.cos(math.pi/4)
    buffer.set_transform_static(t,'test')
    request = NavigateToPose.Goal()
    request.pose.header.frame_id = 'map'
    request.pose.pose.position.x,request.pose.pose.position.y = 10.,21.
    request.pose.pose.orientation.z,request.pose.pose.orientation.w = math.sin(math.pi/4),math.cos(math.pi/4)
    node = SimpleNamespace(calibration=buffer,private=Buffer(),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:request.pose.header.stamp)))
    output = NavigationSession.target(node,request)
    assert output.header.frame_id == 'mocap'
    assert output.pose.position.x == pytest.approx(1.)
    assert output.pose.position.y == pytest.approx(0.,abs=1e-8)
    assert output.pose.orientation.w == pytest.approx(1.)
    request.pose.header.frame_id = 'unknown'
    with pytest.raises(TransformException):
        NavigationSession.target(node,request)


def test_localization_alignment_and_private_rviz():
    assert map_to_odom((4.,5.,math.pi/2),(1.,2.,0.)) == pytest.approx((6.,4.,math.pi/2))
    config = configuration('mur620c','navigation')['Visualization Manager']
    assert config['Global Options']['Fixed Frame'] == 'mocap'
    topics = [display['Topic']['Value'] for display in config['Displays'] if 'Topic' in display]
    assert '/mur620c/navigation/plan' in topics
    assert '/mur620c/navigation/front_scan' in topics
    assert '/mur620c/navigation/rear_scan' in topics


def test_native_cancel_stamp_conversion():
    from mir_driver.mir_bridge import _goal_id_dict_filter, SUB_TOPICS
    ros2 = dict(stamp=dict(sec=0,nanosec=0),id='')
    converted = _goal_id_dict_filter(ros2,False)
    assert converted == dict(stamp=dict(secs=0,nsecs=0),id='')
    assert ros2['stamp'] == dict(sec=0,nanosec=0)
    assert any(t.topic == 'move_base/cancel' and t.dict_filter == _goal_id_dict_filter for t in SUB_TOPICS)


def test_watchdog_stops_output_and_does_not_resume_automatically():
    command = Twist()
    command.linear.x,command.angular.z = 100.,100.
    output = []
    reason = ['']
    node = SimpleNamespace(waiters=[],update_localization=lambda:None,not_ready=lambda:reason[0],
        motion_enabled=True,command=command,command_at=time.monotonic(),settings=DEFAULTS,
        publish_velocity=lambda x,w:output.append((x,w)),hold_zero_until=0.)
    node.halt = lambda:NavigationSession.halt(node)
    NavigationSession.tick(node)
    assert output[-1] == (.25,.4)
    reason[0] = 'Scanner fehlt'
    NavigationSession.tick(node)
    assert output[-1] == (0.,0.)
    assert not node.motion_enabled
    reason[0] = ''
    NavigationSession.tick(node)
    assert output[-1] == (0.,0.)


def test_amcl_navigation_requires_matching_initialized_map(tmp_path):
    node = SimpleNamespace(map_yaml=str(tmp_path/'labor.yaml'),received={})
    report = dict(map_yaml=str(tmp_path/'other.yaml'),initialized_from_mocap_once=True)
    NavigationSession.on_amcl_report(node,String(data=json.dumps(report)))
    assert not node.amcl_map_matches
    report['map_yaml'] = node.map_yaml
    NavigationSession.on_amcl_report(node,String(data=json.dumps(report)))
    assert node.amcl_map_matches
    report['initialized_from_mocap_once'] = False
    NavigationSession.on_amcl_report(node,String(data=json.dumps(report)))
    assert not node.amcl_map_matches


def test_invalid_scanner_stream_does_not_refresh_watchdog():
    delivered = []
    node = SimpleNamespace(robot='mur620c',fresh=lambda _:True,received={},scan_frames={},
        scan_pubs={'front':SimpleNamespace(publish=delivered.append)})
    message = LaserScan()
    message.header.frame_id = 'mur620c/front_laser_link'
    message.angle_increment,message.range_min,message.range_max = .01,.1,10.
    message.ranges = [float('nan'),float('-inf'),-1.]
    NavigationSession.on_scan(node,message,'front')
    assert not delivered and not node.received
    message.ranges = [float('inf')]
    NavigationSession.on_scan(node,message,'front')
    assert delivered and 'front' in node.received
