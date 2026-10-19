"""Gracefully end local legacy mapping launches lacking the end_session service."""

import argparse
import os
from pathlib import Path
import signal
import time

from .settings import ROBOTS


def is_mapping_process(arguments, robot):
    """Match parsed argv and the exact robot; never match shell command strings."""
    for index, argument in enumerate(arguments):
        if Path(argument).name == 'ros2' and arguments[index + 1:index + 4] == [
                'launch', 'match_mur_navigation', 'mapping.launch.py']:
            values = [arg.split(':=', 1)[1] for arg in arguments[index + 4:]
                      if arg.startswith('robot:=')]
            return bool(values) and values[-1] == robot
    executable = next((Path(arg).name for arg in arguments[:2]
                       if Path(arg).name in ('mapping_session', 'mocap_mapper',
                                             'async_slam_toolbox_node')), None)
    module = (len(arguments) >= 3 and Path(arguments[0]).name.startswith('python3')
              and arguments[1:3] == ['-m', 'match_mur_navigation.session'])
    namespace = [arg.split(':=', 1)[1] for arg in arguments if arg.startswith('__ns:=')]
    return bool(executable or module) and bool(namespace) and namespace[-1].strip('/') == f'{robot}/mapping'


def local_processes(robot):
    matches = []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            if directory.stat().st_uid != os.getuid():
                continue
            arguments = directory.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
            if not is_mapping_process(arguments, robot):
                continue
            stat = directory.joinpath('stat').read_text().rsplit(')', 1)[1].split()
            if stat[0] != 'Z':
                matches.append((int(directory.name), stat[19]))
        except (OSError, UnicodeError, IndexError):
            continue
    return matches


def stop_local_session(robot, timeout=7.0):
    if robot not in ROBOTS:
        raise ValueError('Unknown MuR')
    targets = local_processes(robot)
    if not targets:
        raise RuntimeError('Keine lokale Kartierung gefunden. Das externe Backend muss '
                           'den neuen end_session-Dienst anbieten oder auf seinem Rechner beendet werden.')
    for pid, _started in targets:
        # Recheck identity immediately before signalling to avoid a reused PID.
        if (pid, _started) in local_processes(robot):
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not set(targets).intersection(local_processes(robot)):
            return
        time.sleep(0.1)
    raise RuntimeError('Kartierung hat das reguläre Beenden nicht bestätigt; kein erzwungenes Beenden ausgeführt.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', choices=ROBOTS, required=True)
    args = parser.parse_args()
    try:
        stop_local_session(args.robot)
    except (OSError, RuntimeError) as exc:
        parser.exit(1, str(exc) + '\n')
    print(f'Kartierungsprozesse für {args.robot} beendet.')


if __name__ == '__main__':
    main()
