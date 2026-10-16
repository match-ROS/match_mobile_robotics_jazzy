"""Save exactly one retained map snapshot using Nav2, then commit verified files."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import OccupancyGrid
import yaml
from .session import LATCHED
from .settings import map_frame, validate


def verify_export(base, snapshot):
    metadata = yaml.safe_load(base.with_suffix('.yaml').read_text(encoding='utf-8'))
    image = base.parent / metadata['image']
    if not image.is_file() or image.stat().st_size == 0:
        raise RuntimeError('Nav2 did not create the map image')
    expected = [snapshot.info.origin.position.x, snapshot.info.origin.position.y]
    if abs(metadata['resolution'] - snapshot.info.resolution) > 1e-6 or any(
            abs(a-b) > 1e-5 for a, b in zip(metadata['origin'][:2], expected)):
        raise RuntimeError('Export geometry differs from the map snapshot')
    return metadata, image


def save(node, topic, destination, session, overwrite=False):
    executor = node.executor
    owns_executor = executor is None
    if owns_executor:
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)
    try:
        return _save(node, executor, topic, destination, session, overwrite)
    finally:
        if owns_executor:
            executor.remove_node(node)
            executor.shutdown()


def _save(node, executor, topic, destination, session, overwrite):
    robot, mode = session['robot'], session['mode']
    settings = validate(robot, mode, session['settings'])
    base = Path(destination).expanduser().absolute()
    if base.suffix in ('.yaml', '.pgm'):
        base = base.with_suffix('')
    outputs = [Path(str(base) + suffix) for suffix in ('.yaml', '.pgm', '.mapping.yaml')]
    if any(p.exists() for p in outputs) and not overwrite:
        raise FileExistsError('Destination exists; choose another name or confirm overwrite')
    snapshot = []
    sub = node.create_subscription(OccupancyGrid, topic, lambda msg: snapshot.append(msg) if not snapshot else None, LATCHED)
    deadline = time.monotonic() + 5
    while not snapshot and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    node.destroy_subscription(sub)
    if not snapshot:
        raise RuntimeError('No retained map received')
    msg = snapshot[0]
    if msg.header.frame_id != map_frame(robot, mode) or not msg.data or len(msg.data) != msg.info.width * msg.info.height:
        raise RuntimeError('Invalid map snapshot or wrong frame')
    snapshot_topic = '/mur_mapping_export_' + uuid.uuid4().hex
    pub = node.create_publisher(OccupancyGrid, snapshot_topic, LATCHED)
    pub.publish(msg)
    base.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.mur-map-', dir=base.parent) as directory:
        stage = Path(directory) / 'map'
        command = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(stage),
                   '--fmt', 'pgm', '--mode', 'trinary', '--free', '0.25', '--occ', '0.65',
                   '--ros-args', '-r', f'map:={snapshot_topic}', '-p', 'save_map_timeout:=5.0',
                   '-p', 'map_subscribe_transient_local:=true']
        process = subprocess.Popen(command, start_new_session=True)
        try:
            deadline = time.monotonic() + 15
            while process.poll() is None and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            if process.poll() is None:
                raise TimeoutError('Nav2 map saver timed out')
            if process.returncode:
                raise RuntimeError(f'Nav2 map saver exited with {process.returncode}')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_publisher(pub)
        metadata, image = verify_export(stage, msg)
        metadata['image'] = outputs[1].name
        stage.with_suffix('.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
        companion = dict(robot=robot, mode=mode, frame=msg.header.frame_id, settings=settings,
                         pose_topic=f'/qualisys/{robot}/pose' if mode == 'mocap' else f'/{robot}/odom',
                         stamp=dict(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec))
        sidecar = Path(directory) / 'map.mapping.yaml'
        sidecar.write_text(yaml.safe_dump(companion), encoding='utf-8')
        # Recheck just before writing; never silently replace a file created while saving.
        if not overwrite and any(p.exists() for p in outputs):
            raise FileExistsError('Destination appeared while saving')
        os.replace(image, outputs[1])
        os.replace(sidecar, outputs[2])
        os.replace(stage.with_suffix('.yaml'), outputs[0])
    return outputs


def main():
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--overwrite', action='store_true')
    parser.add_argument('--topic', help='Retained snapshot topic, including a GUI-owned cache')
    args = parser.parse_args()
    session = yaml.safe_load(Path(args.session).read_text(encoding='utf-8'))
    rclpy.init(args=[])
    node = rclpy.create_node('mur_mapping_export_' + uuid.uuid4().hex[:8])
    try:
        outputs = save(node, args.topic or f"/{session['robot']}/mapping/map", args.output, session, args.overwrite)
        print(json.dumps({'saved': [str(p) for p in outputs]}), flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
