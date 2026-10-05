from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import xacro
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE/'config/calibrated_markers.yaml'


def render(tmp_path, robot):
    wrapper = tmp_path/'markers.xacro'
    wrapper.write_text(f'''<robot xmlns:xacro="http://wiki.ros.org/xacro" name="test">
      <link name="base_link"/>
      <xacro:include filename="{PACKAGE}/urdf/calibrated_markers.xacro"/>
      <xacro:calibrated_marker_frames robot_name="{robot}" config_file="{CONFIG}"/>
    </robot>''')
    return ET.fromstring(xacro.process_file(str(wrapper)).toxml())


def test_calibrated_transform_direction_and_rotation(tmp_path):
    root = render(tmp_path, 'mur620a')
    markers = yaml.safe_load(CONFIG.read_text())['robots']['mur620a']['markers']
    assert len(root.findall('joint')) == 2
    assert {link.attrib['name'] for link in root.findall('link')} == {
        'base_link', 'aruco_rear_left', 'aruco_rear_right'}
    for name, data in markers.items():
        joint = root.find(f"joint[@name='base_link_to_aruco_{name}']")
        if not data.get('publish_tf', True):
            assert joint is None
            continue
        assert joint.attrib['type'] == 'fixed'
        assert joint.find('parent').attrib['link'] == 'base_link'
        assert joint.find('child').attrib['link'] == 'aruco_'+name
        origin = joint.find('origin')
        xyz = np.fromstring(origin.attrib['xyz'], sep=' ')
        rpy = np.fromstring(origin.attrib['rpy'], sep=' ')
        np.testing.assert_allclose(xyz, data['translation'], atol=1e-12)
        np.testing.assert_allclose(Rotation.from_euler('xyz', rpy).as_matrix(),
                                   Rotation.from_quat(data['quaternion_xyzw']).as_matrix(), atol=1e-12)
        assert xyz[0] < -.6  # These are the measured rear markers.
    assert markers['rear_left']['id'] == 0 and markers['rear_right']['id'] == 1
    assert markers['rear_left']['translation'][2] == .58


def test_front_draft_frames_are_disabled_until_independent_validation(tmp_path):
    root = render(tmp_path, 'mur620a')
    markers = yaml.safe_load(CONFIG.read_text())['robots']['mur620a']['markers']
    for name, mid, sign in [('front_left', 24, 1), ('front_right', 7, -1)]:
        marker = markers[name]
        assert marker['id'] == mid
        assert marker['quality'] == 'draft'
        assert marker['publish_tf'] is False
        assert marker['translation'][0] > 0
        assert marker['translation'][1]*sign > 0
        assert root.find(f"link[@name='aruco_{name}']") is None


def test_front_frames_can_be_rendered_for_explicit_review(tmp_path):
    config = yaml.safe_load(CONFIG.read_text())
    for marker in config['robots']['mur620a']['markers'].values():
        marker['publish_tf'] = True
    file = tmp_path/'review_config.yaml'
    file.write_text(yaml.safe_dump(config))
    wrapper = tmp_path/'review.xacro'
    wrapper.write_text(f'''<robot xmlns:xacro="http://wiki.ros.org/xacro" name="review">
      <link name="base_link"/>
      <xacro:include filename="{PACKAGE}/urdf/calibrated_markers.xacro"/>
      <xacro:calibrated_marker_frames robot_name="mur620a" config_file="{file}"/>
    </robot>''')
    root = ET.fromstring(xacro.process_file(str(wrapper)).toxml())
    for name in ('front_left', 'front_right'):
        data = config['robots']['mur620a']['markers'][name]
        joint = root.find(f"joint[@name='base_link_to_aruco_{name}']")
        assert joint is not None
        origin = joint.find('origin')
        xyz = np.fromstring(origin.attrib['xyz'], sep=' ')
        rpy = np.fromstring(origin.attrib['rpy'], sep=' ')
        np.testing.assert_allclose(xyz, data['translation'], atol=1e-12)
        np.testing.assert_allclose(Rotation.from_euler('xyz', rpy).as_matrix(),
                                   Rotation.from_quat(data['quaternion_xyzw']).as_matrix(), atol=1e-12)


@pytest.mark.parametrize('robot', ['mur620b', 'mur620d', ''])
def test_calibration_only_applies_to_measured_robot(tmp_path, robot):
    root = render(tmp_path, robot)
    assert not root.findall('joint')
    assert len(root.findall('link')) == 1


def test_hardware_description_includes_marker_frames():
    root = ET.fromstring(xacro.process_file(
        str(PACKAGE/'urdf/mur_620.gazebo.xacro'),
        mappings={'robot_namespace': 'mur620a', 'use_sim': 'false', 'use_arms': 'false'}).toxml())
    assert root.find("joint[@name='base_link_to_aruco_rear_left']") is not None
    assert root.find("joint[@name='base_link_to_aruco_rear_right']") is not None
