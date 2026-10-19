"""Generate a robot-specific RViz config and forward shutdown to RViz."""
import argparse
import signal
import subprocess
import tempfile
from pathlib import Path
import yaml
from .settings import ROBOTS, map_frame


def configuration(robot, mode):
    ns = (f'/{robot}' if mode == 'mir' else
          f'/{robot}/' + (mode if mode in ('localization','navigation') else 'mapping'))
    def topic(name, latched=False, reliable=False):
        return {'Value': name, 'Depth': 5, 'Reliability Policy': 'Reliable' if latched or reliable else 'Best Effort',
                'Durability Policy': 'Transient Local' if latched else 'Volatile', 'History Policy': 'Keep Last'}
    displays = [
        {'Class': 'rviz_default_plugins/Grid', 'Name': 'Raster', 'Enabled': True, 'Cell Size': 1.0},
        {'Class': 'rviz_default_plugins/Map', 'Name': 'Karte', 'Enabled': True,
         'Topic': topic(ns + '/map', True), 'Color Scheme': 'map', 'Alpha': 0.8},
        {'Class': 'rviz_default_plugins/RobotModel', 'Name': robot, 'Enabled': True,
         'Description Source': 'Topic', 'Description Topic': topic(f'/{robot}/robot_description', True),
         'TF Prefix': robot},
        {'Class': 'rviz_default_plugins/TF', 'Name': 'TF', 'Enabled': False},
    ]
    scan_names = (('f_scan', 'b_scan') if mode == 'mir' else
                  ('front_scan', 'rear_scan') if mode in ('mocap','navigation') else ('scan',))
    for name in scan_names:
        displays.append({'Class': 'rviz_default_plugins/LaserScan', 'Name': name,
                         'Enabled': True, 'Topic': topic(ns + '/' + name), 'Size (m)': 0.03,
                         'Queue Size': 50})
    if mode == 'navigation':
        for topic_name,label,kind in (
            ('plan','Globaler Pfad','Path'),
            ('local_costmap/costmap','Lokale Hinderniskarte','Map'),
            ('global_costmap/costmap','Globale Hinderniskarte','Map')):
            displays.append({'Class':'rviz_default_plugins/'+kind,'Name':label,'Enabled':True,
                             'Topic':topic(ns+'/'+topic_name,kind=='Map')})
    if mode == 'localization':
        displays.extend([
            {'Class': 'rviz_default_plugins/PoseWithCovariance', 'Name': 'AMCL-Pose',
             'Enabled': True, 'Topic': topic(ns+'/amcl_pose', True), 'Color': '255; 80; 30'},
            {'Class': 'rviz_default_plugins/Pose', 'Name': 'Mocap-Referenz',
             'Enabled': True, 'Topic': topic(ns+'/mocap_pose'), 'Color': '40; 210; 90'}])
    elif mode in ('mocap','navigation'):
        displays.append({'Class': 'rviz_default_plugins/Pose', 'Name': 'Mocap-Pose',
                         'Enabled': True, 'Topic': topic(ns + '/robot_pose')})
    elif mode != 'mir':
        displays.append({'Class': 'rviz_default_plugins/PoseWithCovariance', 'Name': 'SLAM-Pose',
                         'Enabled': True, 'Topic': topic(ns + '/slam_pose')})
    tools = [{'Class': 'rviz_default_plugins/MoveCamera'},
             {'Class': 'rviz_default_plugins/Select'},
             {'Class': 'rviz_default_plugins/FocusCamera'}]
    if mode in ('navigation', 'mir'):
        goal_topic = f'/{robot}/navigation/' + ('mir_goal_pose' if mode == 'mir' else 'goal_pose')
        tools.append({'Class': 'rviz_default_plugins/SetGoal', 'Topic': topic(goal_topic, reliable=True)})
        displays.append({'Class': 'rviz_default_plugins/Pose', 'Name': 'Klickziel',
                         'Enabled': True, 'Topic': topic(goal_topic)})
    return {'Panels': [{'Class': 'rviz_common/Displays', 'Name': 'Displays'}],
            'Visualization Manager': {'Class': '', 'Global Options': {'Fixed Frame':
                'map' if mode == 'mir' else 'mocap' if mode in ('localization','navigation') else map_frame(robot, mode)},
                'Tools': tools,
                'Displays': displays, 'Views': {'Current': {'Class': 'rviz_default_plugins/TopDownOrtho',
                'Name': 'Draufsicht', 'Scale': 50, 'Target Frame': '<Fixed Frame>'}}}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=ROBOTS, required=True)
    parser.add_argument('--mode', choices=['mocap', 'slam', 'localization','navigation','mir'], required=True)
    parser.add_argument('--sim', action='store_true')
    # launch_ros adds --ros-args; this executable itself has no ROS node.
    args, _ = parser.parse_known_args()
    with tempfile.TemporaryDirectory(prefix='mur_mapping_rviz_') as directory:
        path = Path(directory) / 'mapping.rviz'
        path.write_text(yaml.safe_dump(configuration(args.robot, args.mode)), encoding='utf-8')
        ns = f'/{args.robot}/' + (args.mode if args.mode in ('localization','navigation') else 'mapping')
        arguments = ['rviz2', '-d', str(path), '--ros-args']
        if args.mode != 'mir':
            arguments += ['-r', f'/tf:={ns}/tf', '-r', f'/tf_static:={ns}/tf_static']
        arguments += ['-p', f'use_sim_time:={str(args.sim).lower()}']
        process = subprocess.Popen(arguments)
        def stop(signum, _frame):
            if process.poll() is None:
                process.send_signal(signum)
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        try:
            return process.wait()
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == '__main__':
    raise SystemExit(main())
