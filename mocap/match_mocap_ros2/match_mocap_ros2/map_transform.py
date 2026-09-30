"""Checked ROS 1 map -> mocap calibration and pose conversion."""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class MapToMocap:
    x: float
    y: float
    z: float
    yaw: float

    @classmethod
    def from_ros1_launch(cls, launch_xml):
        try:
            root = ET.fromstring(launch_xml)
        except ET.ParseError as exc:
            raise ValueError('Invalid ROS 1 launch XML') from exc
        matches = [
            node for node in root.iter('node')
            if node.get('name') == 'map_to_mocap'
            and node.get('pkg') == 'tf'
            and node.get('type') == 'static_transform_publisher'
        ]
        if len(matches) != 1:
            raise ValueError('Expected exactly one ROS 1 map_to_mocap static transform')
        args = matches[0].get('args', '').split()
        if len(args) != 9 or args[6:8] != ['map', 'mocap']:
            raise ValueError('Expected x y z yaw pitch roll map mocap period in ROS 1 launch')
        x, y, z, yaw, pitch, roll = (float(value) for value in args[:6])
        if not all(math.isfinite(value) for value in (x, y, z, yaw, pitch, roll)):
            raise ValueError('Non-finite map_to_mocap transform')
        if abs(pitch) > 1e-9 or abs(roll) > 1e-9:
            raise ValueError('Only a planar map_to_mocap transform is supported')
        if not math.isfinite(float(args[8])) or float(args[8]) <= 0:
            raise ValueError('Invalid map_to_mocap publication period')
        return cls(x, y, z, yaw)

    def apply(self, pose):
        if pose.header.frame_id != 'mocap':
            raise ValueError(f'Expected source frame mocap, got {pose.header.frame_id!r}')
        source = pose.pose.position
        orientation = pose.pose.orientation
        rotation = Rotation.from_euler('z', self.yaw)
        point = rotation.apply([source.x, source.y, source.z])
        quat = (rotation * Rotation.from_quat([
            orientation.x, orientation.y, orientation.z, orientation.w,
        ])).as_quat()
        result = PoseStamped()
        result.header.stamp = pose.header.stamp
        result.header.frame_id = 'map'
        result.pose.position.x = self.x + point[0]
        result.pose.position.y = self.y + point[1]
        result.pose.position.z = self.z + point[2]
        result.pose.orientation.x, result.pose.orientation.y = quat[:2]
        result.pose.orientation.z, result.pose.orientation.w = quat[2:]
        return result


# Measured ROS 1 calibration on roscore, 2026-09-30. A repository checkout can
# otherwise silently restore the older, substantially different map origin.
VERIFIED_MAP_TO_MOCAP = MapToMocap(48.2795, 43.2532, 0.0, 1.290419)


def checked_map_transform(launch_xml):
    transform = MapToMocap.from_ros1_launch(launch_xml)
    if any(
        abs(getattr(transform, field) - getattr(VERIFIED_MAP_TO_MOCAP, field)) > 1e-6
        for field in ('x', 'y', 'z', 'yaw')
    ):
        raise ValueError(
            'roscore map_to_mocap differs from the verified calibration; '
            'review both values before publishing map poses'
        )
    return transform
