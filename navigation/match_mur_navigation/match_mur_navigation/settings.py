"""Shared session validation and configuration; no ROS or Qt dependency."""
import math

ROBOTS = ('mur620a', 'mur620b', 'mur620c', 'mur620d')
DEFAULTS = dict(resolution=0.02, min_range=0.1, max_range=10.0,
                map_update_interval=1.0, mocap_time_offset=0.0,
                front_time_offset=0.0, rear_time_offset=0.0,
                pose_timeout=0.5, max_pose_gap=0.1, max_cells=25000000)


def validate(robot, mode, settings):
    if robot not in ROBOTS or mode not in ('mocap', 'slam'):
        raise ValueError('Unknown MuR or mapping mode')
    result = dict(DEFAULTS)
    unknown = set(settings) - set(DEFAULTS)
    if unknown:
        raise ValueError(f'Unknown settings: {sorted(unknown)}')
    result.update(settings)
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in result.values()):
        raise ValueError('Mapping settings must be finite numbers')
    if not (0.005 <= result['resolution'] <= 0.2 and
            0 <= result['min_range'] < result['max_range'] <= 100 and
            0.1 <= result['map_update_interval'] <= 30 and
            0 < result['max_pose_gap'] <= result['pose_timeout'] <= 2 and
            4096 <= result['max_cells'] <= 100000000 and
            int(result['max_cells']) == result['max_cells']):
        raise ValueError('Mapping settings outside supported bounds')
    if any(abs(result[k]) > 1 for k in ('mocap_time_offset', 'front_time_offset', 'rear_time_offset')):
        raise ValueError('Time corrections must be between -1 and +1 seconds')
    result['max_cells'] = int(result['max_cells'])
    return result


def map_frame(robot, mode):
    return 'mocap' if mode == 'mocap' else f'{robot}/mapping/map'


def local_transform(transform, robot):
    """Keep only robot-local edges; root pose and odometry belong to this session."""
    parent = transform.header.frame_id.strip('/')
    child = transform.child_frame_id.strip('/')
    prefix = robot + '/'
    if parent == prefix + 'base_footprint' and child == prefix + 'base_link':
        return True
    return (parent.startswith(prefix) and child.startswith(prefix) and
            parent != prefix + 'odom' and
            child not in (prefix + 'odom', prefix + 'base_footprint', prefix + 'base_link'))
