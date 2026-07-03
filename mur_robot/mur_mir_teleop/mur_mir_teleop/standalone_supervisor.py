#!/usr/bin/env python3
"""Supervisor for standalone DS4 MiR teleoperation."""

import argparse
import glob
import os
import re
import signal
import subprocess
import sys
import time

import yaml


DEFAULT_CONFIG = '/etc/mur-standalone/mir_teleop.yaml'


def load_config(path):
    with open(path, 'r', encoding='utf-8') as config_file:
        data = yaml.safe_load(config_file) or {}
    return data


def nested(config, *keys, default=None):
    current = config
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def joystick_id(path):
    match = re.search(r'/js([0-9]+)$', path)
    if not match:
        return None
    return int(match.group(1))


def joystick_name(path):
    base = os.path.basename(path)
    name_path = os.path.join('/sys/class/input', base, 'device', 'name')
    try:
        with open(name_path, 'r', encoding='utf-8') as name_file:
            return name_file.read().strip()
    except OSError:
        return ''


def joystick_uniq(path):
    base = os.path.basename(path)
    uniq_path = os.path.join('/sys/class/input', base, 'device', 'uniq')
    try:
        with open(uniq_path, 'r', encoding='utf-8') as uniq_file:
            return uniq_file.read().strip()
    except OSError:
        return ''


def normalize_address(address):
    return str(address).strip().upper().replace('-', ':')


def connect_known_controllers(addresses):
    for address in addresses:
        normalized = normalize_address(address)
        if not normalized:
            continue
        try:
            subprocess.run(
                ['bluetoothctl', 'connect', normalized],
                check=False,
                timeout=5.0,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except (OSError, subprocess.TimeoutExpired):
            pass


def ros_executable(package, executable):
    search_paths = os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    search_paths.extend(['/opt/ros/jazzy'])
    for prefix in search_paths:
        if not prefix:
            continue
        candidate = os.path.join(prefix, 'lib', package, executable)
        if os.path.exists(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return executable


def ros_service_available(service_name, env):
    try:
        result = subprocess.run(
            ['ros2', 'service', 'type', service_name],
            check=False,
            timeout=2.0,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


def find_matching_joystick(patterns, known_addresses):
    known = {normalize_address(address) for address in known_addresses if normalize_address(address)}
    devices = sorted(glob.glob('/dev/input/js*'))
    for device in devices:
        name = joystick_name(device)
        if not any(pattern in name for pattern in patterns):
            continue
        uniq = normalize_address(joystick_uniq(device))
        if known and uniq not in known:
            continue
        device_id = joystick_id(device)
        if device_id is not None:
            return device, device_id, name
    return None, None, None


class ProcessGroup:
    def __init__(self):
        self.processes = []

    def start(self, label, command, env):
        print(f'[mur_mir_standalone] starting {label}: {" ".join(command)}', flush=True)
        process = subprocess.Popen(command, env=env, start_new_session=True)
        self.processes.append((label, process))
        return process

    def any_exited(self):
        for label, process in self.processes:
            code = process.poll()
            if code is not None:
                return label, code
        return None, None

    def stop(self):
        for label, process in reversed(self.processes):
            if process.poll() is None:
                print(f'[mur_mir_standalone] stopping {label}', flush=True)
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 5.0
        for _label, process in reversed(self.processes):
            remaining = max(0.1, deadline - time.monotonic())
            try:
                process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        for _label, process in reversed(self.processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self.processes = []


def teleop_params(config):
    params = nested(config, 'teleop', default={}) or {}
    result = []
    for key, value in params.items():
        result.extend(['-p', f'{key}:={value}'])
    return result


def run_supervisor(config):
    robot_name = str(config.get('robot_name', 'mur620d')).strip('/')
    ros_domain_id = str(config.get('ros_domain_id', os.environ.get('ROS_DOMAIN_ID', '62')))
    patterns = nested(
        config,
        'controller',
        'name_patterns',
        default=['Wireless Controller', 'Sony Interactive Entertainment Wireless Controller', 'DUALSHOCK 4'],
    )
    known_addresses = nested(config, 'controller', 'bluetooth_addresses', default=[]) or []
    connect_interval = float(nested(config, 'controller', 'bluetooth_connect_interval_sec', default=5.0))
    poll_interval = float(nested(config, 'controller', 'poll_interval_sec', default=1.0))
    mir_hostname = str(config.get('mir_hostname', '192.168.12.20'))
    mir_port = str(config.get('mir_port', 9090))
    mir_type = str(config.get('mir_type', 'mir_600'))
    joy_deadzone = str(nested(config, 'joy', 'deadzone', default=0.05))
    joy_autorepeat = str(nested(config, 'joy', 'autorepeat_rate', default=20.0))

    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = ros_domain_id
    env['ROS2CLI_NO_DAEMON'] = '1'
    env.setdefault('ROS_LOG_DIR', '/opt/mur-standalone/log/ros')
    env.setdefault('PYTHONUNBUFFERED', '1')
    env.setdefault('RCUTILS_LOGGING_BUFFERED_STREAM', '0')
    os.makedirs(env['ROS_LOG_DIR'], exist_ok=True)
    last_connect_attempt = 0.0

    while True:
        now = time.monotonic()
        if known_addresses and now - last_connect_attempt >= connect_interval:
            connect_known_controllers(known_addresses)
            last_connect_attempt = now

        device, device_id, name = find_matching_joystick(patterns, known_addresses)
        if device is None:
            print(
                '[mur_mir_standalone] waiting for paired DS4 controller '
                f'(patterns={patterns})',
                flush=True,
            )
            time.sleep(poll_interval)
            continue

        print(
            f'[mur_mir_standalone] using joystick {device} id={device_id} name="{name}"',
            flush=True,
        )
        group = ProcessGroup()
        try:
            bridge_ready_service = f'/{robot_name}/mir_bridge_ready'
            if ros_service_available(bridge_ready_service, env):
                print(
                    f'[mur_mir_standalone] using existing MiR bridge at {bridge_ready_service}',
                    flush=True,
                )
            else:
                group.start(
                    'mir_bridge',
                    [
                        ros_executable('mir_driver', 'mir_bridge'),
                        '--ros-args',
                        '-r',
                        f'__ns:=/{robot_name}',
                        '-p',
                        f'hostname:={mir_hostname}',
                        '-p',
                        f'port:={mir_port}',
                        '-p',
                        f'mir_type:={mir_type}',
                        '-p',
                        'enabled_pub_topics:=b_raw_scan b_scan f_raw_scan f_scan scan robot_pose map map_metadata odom odom_enc tf tf_static',
                        '-p',
                        f'tf_prefix:={robot_name}',
                    ],
                    env,
                )
            group.start(
                'joy_node',
                [
                    ros_executable('joy', 'joy_node'),
                    '--ros-args',
                    '-r',
                    f'__ns:=/{robot_name}',
                    '-p',
                    f'device_id:={device_id}',
                    '-p',
                    f'device_name:={name}',
                    '-p',
                    f'deadzone:={joy_deadzone}',
                    '-p',
                    f'autorepeat_rate:={joy_autorepeat}',
                ],
                env,
            )
            group.start(
                'ds4_mir_teleop',
                [
                    ros_executable('mur_mir_teleop', 'ds4_mir_teleop'),
                    '--ros-args',
                    *teleop_params(config),
                ],
                env,
            )

            while True:
                label, code = group.any_exited()
                if label is not None:
                    print(
                        f'[mur_mir_standalone] {label} exited with code {code}; restarting stack',
                        flush=True,
                    )
                    break
                if not os.path.exists(device):
                    print('[mur_mir_standalone] joystick disappeared; restarting stack', flush=True)
                    break
                time.sleep(0.5)
        finally:
            group.stop()
            time.sleep(1.0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=DEFAULT_CONFIG)
    args = parser.parse_args(argv)
    config = load_config(args.config)
    try:
        run_supervisor(config)
    except KeyboardInterrupt:
        return 0
    return 1


if __name__ == '__main__':
    sys.exit(main())
