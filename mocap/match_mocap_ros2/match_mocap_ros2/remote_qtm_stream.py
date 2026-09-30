"""Run on the ROS 1 host, where the Qualisys qtm Python SDK is installed.

The local ROS 2 node sends this file to ``ssh roscore python3 -u -`` over stdin.
No files or ROS packages need to be installed on roscore.
"""

import asyncio
import json
import math
import signal
import sys
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import qtm


MAP_LAUNCH_PATH = Path('/home/rosmatch/catkin_ws/src/match_mocap/launch_mocap/launch/mocap_launch.launch')


async def main():
    # Exit with the SSH channel when the local ROS 2 node closes it.
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)
    host, port, frequency = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    connection = await qtm.connect(host, port=port, version='1.23', timeout=5)
    if connection is None:
        raise RuntimeError('QTM connection failed')

    try:
        parameters = ET.fromstring(await connection.get_parameters(parameters=['6d']))
        body_names = [node.text for node in parameters.findall('.//Body/Name')]
        config = {'kind': 'config', 'body_names': body_names}
        try:
            config['map_launch_xml'] = MAP_LAUNCH_PATH.read_text(encoding='utf-8')
        except OSError as exc:
            config['map_launch_error'] = str(exc)
        print(json.dumps(config), flush=True)

        last_packet_at = time.monotonic()

        def on_packet(packet):
            nonlocal last_packet_at
            last_packet_at = time.monotonic()
            _, bodies = packet.get_6d()
            tracked = []
            for name, (position, rotation) in zip(body_names, bodies):
                values = (position.x, position.y, position.z, *rotation.matrix)
                if all(math.isfinite(value) for value in values):
                    tracked.append({
                        'name': name,
                        'position_mm': [position.x, position.y, position.z],
                        'rotation_column_major': list(rotation.matrix),
                    })
            print(json.dumps({
                'kind': 'frame',
                'frame_number': packet.framenumber,
                'bodies': tracked,
            }), flush=True)

        await connection.stream_frames(
            frames='frequency:%d' % frequency,
            components=['6d'],
            on_packet=on_packet,
        )
        while True:
            await asyncio.sleep(1)
            if time.monotonic() - last_packet_at > 10:
                raise RuntimeError('No QTM frames received for 10 seconds')
    finally:
        connection.disconnect()


if __name__ == '__main__':
    asyncio.run(main())
