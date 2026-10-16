import copy
import math
import uuid

import pytest
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from match_mur_navigation.session import MappingSession, corrected
from match_mur_navigation.settings import local_transform, validate


def transform(parent, child):
    msg = TransformStamped(child_frame_id=child)
    msg.header.frame_id = parent
    msg.transform.rotation.w = 1.0
    return msg


def test_private_tree_filters_global_and_root_edges():
    robot = 'mur620d'
    assert local_transform(transform(robot+'/base_link', robot+'/front_laser_link'), robot)
    assert local_transform(transform(robot+'/base_footprint', robot+'/base_link'), robot)
    for parent, child in [('map', robot+'/base_footprint'), ('mocap', robot+'/base_link'),
                          (robot+'/odom', robot+'/base_footprint'), ('map', robot+'/odom'),
                          ('mur620a/base_link', 'mur620a/front_laser_link')]:
        assert not local_transform(transform(parent, child), robot)


def test_offsets_and_validation():
    msg = PoseStamped()
    msg.header.stamp.sec, msg.header.stamp.nanosec = 10, 50000000
    result = corrected(msg, -0.1)
    assert result.header.stamp.sec == 9
    assert result.header.stamp.nanosec == 950000000
    assert msg.header.stamp.sec == 10
    for settings in ({'resolution': 0}, {'min_range': 20}, {'max_range': math.nan},
                     {'mocap_time_offset': 2}, {'unknown': 1}):
        with pytest.raises(ValueError):
            validate('mur620d', 'mocap', settings)


class Capture:
    def __init__(self):
        self.values = []
    def publish(self, value):
        self.values.append(copy.deepcopy(value))


@pytest.fixture
def session():
    context = rclpy.context.Context()
    rclpy.init(args=[], context=context)
    node = MappingSession(context=context, namespace='/test_'+uuid.uuid4().hex[:8],
                          parameter_overrides=[Parameter('robot', value='mur620d')])
    yield node
    node.destroy_node()
    rclpy.shutdown(context=context)


def test_full_pose_forwarding_no_global_publication_and_stop(session):
    node = session
    node.pose_pub = Capture()
    node.pose_display_pub = Capture()
    node.tf_pub = Capture()
    node.static_pub = Capture()
    pose = PoseStamped()
    pose.header.frame_id = 'mocap'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.z = 0.25
    pose.pose.orientation.x = math.sin(0.1)
    pose.pose.orientation.w = math.cos(0.1)
    node.on_pose(pose)
    assert node.tf_pub.values[-1].transforms[0].transform.translation.z == 0.25
    assert node.tf_pub.values[-1].transforms[0].transform.rotation.x == pose.pose.orientation.x
    node.on_static(TFMessage(transforms=[transform('map', 'mur620d/base_footprint'),
        transform('mur620d/base_link', 'mur620d/front_laser_link')]))
    assert len(node.static_pub.values[-1].transforms) == 1
    scan = LaserScan()
    scan.header.stamp = node.get_clock().now().to_msg()
    node.scan_pubs['front_scan'] = Capture()
    node.on_scan(scan, 'front_scan', 'front_time_offset')
    assert len(node.scan_pubs['front_scan'].values) == 1
    response = node.set_recording(SetBool.Request(data=False), SetBool.Response())
    assert response.success
    node.on_scan(scan, 'front_scan', 'front_time_offset')
    assert len(node.scan_pubs['front_scan'].values) == 1
    assert not node.set_recording(SetBool.Request(data=True), SetBool.Response()).success
    # No publisher in this session writes hardware TF topics.
    for endpoint in node.get_publishers_info_by_topic('/tf') + node.get_publishers_info_by_topic('/tf_static'):
        assert endpoint.node_name != node.get_name()
