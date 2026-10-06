#!/usr/bin/env python3
import http.client
import json
import math
import threading
import time

import websocket
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32

DEFAULT_AUTH = (
    'Basic '
    'ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='
)


def _normalize_host(hostname):
    parsed = urlparse(hostname if '://' in hostname else 'http://' + hostname)
    return parsed.hostname or hostname


class MiRBatteryStatePublisher(Node):
    def __init__(self):
        super().__init__('mir_battery_state_publisher')
        self.robot_ip = self.declare_parameter('mir_hostname', '192.168.12.20').value
        self.auth = self.declare_parameter('mir_restapi_auth', DEFAULT_AUTH).value
        self.period = float(self.declare_parameter('period', 2.0).value)
        self.timeout = float(self.declare_parameter('timeout', 3.0).value)
        self.failed_reads = 0
        self.publisher = self.create_publisher(BatteryState, 'battery_state', 1)
        self.remaining_publisher = self.create_publisher(Int32, 'battery_time_remaining', 1)
        self.bms_port = int(self.declare_parameter('mir_port', 9090).value)
        self._bms_lock = threading.Lock()
        self._bms_data = None
        self._bms_received_at = 0.0
        self._stop_bms = threading.Event()
        self._bms_socket = None
        self._bms_thread = threading.Thread(target=self._stream_bms, daemon=True)
        self._bms_thread.start()
        self.timer = self.create_timer(self.period, self.query_status)

    def _stream_bms(self):
        url = f'ws://{_normalize_host(str(self.robot_ip))}:{self.bms_port}/'
        while not self._stop_bms.is_set():
            def on_open(ws):
                ws.send(json.dumps({
                    'op': 'subscribe', 'topic': '/PB/bms_status',
                    'throttle_rate': 1000, 'queue_length': 1,
                }))

            def on_message(_ws, message):
                try:
                    packet = json.loads(message)
                    if packet.get('op') != 'publish' or packet.get('topic') != '/PB/bms_status':
                        return
                    data = packet['msg']
                    with self._bms_lock:
                        self._bms_data = data
                        self._bms_received_at = time.monotonic()
                except (KeyError, TypeError, ValueError):
                    return

            ws = websocket.WebSocketApp(url, on_open=on_open, on_message=on_message)
            self._bms_socket = ws
            try:
                ws.run_forever(ping_interval=20, ping_timeout=5)
            except Exception as exc:
                self.get_logger().warning(f'MiR BMS stream failed: {exc}',
                                          throttle_duration_sec=10.0)
            finally:
                self._bms_socket = None
            self._stop_bms.wait(2.0)

    def _recent_bms(self):
        with self._bms_lock:
            if self._bms_data is None or time.monotonic() - self._bms_received_at > 6.0:
                return None
            return dict(self._bms_data)

    @staticmethod
    def _bms_float(data, key, scale=1.0):
        try:
            value = float(data[key]) * scale
            return value if math.isfinite(value) else math.nan
        except (KeyError, TypeError, ValueError):
            return math.nan

    def _fill_bms_fields(self, msg, data):
        if data is None:
            msg.voltage = math.nan
            msg.current = math.nan
            msg.charge = math.nan
            msg.capacity = math.nan
            msg.temperature = math.nan
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            return
        msg.voltage = self._bms_float(data, 'pack_voltage')
        charge_current = self._bms_float(data, 'charge_current')
        discharge_current = self._bms_float(data, 'discharge_current')
        msg.current = (charge_current - discharge_current if
                       math.isfinite(charge_current) and math.isfinite(discharge_current)
                       else math.nan)
        msg.charge = self._bms_float(data, 'remaining_capacity', 0.001)
        msg.capacity = self._bms_float(data, 'full_capacity', 0.001)
        msg.temperature = self._bms_float(data, 'temperature')
        charging = bool(data.get('status_charging'))
        discharging = bool(data.get('status_discharging'))
        if charging and (not discharging or msg.current > 0.1):
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif discharging and (not charging or msg.current < -0.1):
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        elif data.get('status_fully_charged'):
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif math.isfinite(msg.current) and abs(msg.current) < 0.1:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN

    def query_status(self):
        headers = {
            'Authorization': self.auth,
            'Accept': 'application/json',
            'Accept-Language': 'en_US',
        }
        host = _normalize_host(str(self.robot_ip))
        connection = http.client.HTTPConnection(host=host, port=80, timeout=self.timeout)
        try:
            connection.request('GET', '/api/v2.0.0/status', headers=headers)
            response = connection.getresponse()
            if response.status < 200 or response.status >= 300:
                raise RuntimeError('HTTP {} {}'.format(response.status, response.reason))
            data = json.loads(response.read().decode('utf-8'))
            percentage = float(data.get('battery_percentage', math.nan))
            if not math.isfinite(percentage) or not 0.0 <= percentage <= 100.0:
                raise ValueError('MiR battery_percentage is unavailable')

            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.percentage = round(percentage / 100.0, 4)
            msg.present = True
            self._fill_bms_fields(msg, self._recent_bms())
            self.publisher.publish(msg)
            try:
                remaining = int(data['battery_time_remaining'])
                if 0 <= remaining <= 2_147_483_647:
                    self.remaining_publisher.publish(Int32(data=remaining))
            except (KeyError, TypeError, ValueError):
                pass
            self.failed_reads = 0
            self.get_logger().info('[{}] Battery: {:.2f}%'.format(host, percentage), throttle_duration_sec=10.0)
        except Exception as exc:
            self.failed_reads += 1
            message = 'Battery read failed from {}: {}'.format(host, exc)
            if self.failed_reads < 2:
                self.get_logger().debug(message)
            else:
                self.get_logger().warning(message, throttle_duration_sec=10.0)
        finally:
            connection.close()

    def destroy_node(self):
        self._stop_bms.set()
        if self._bms_socket is not None:
            self._bms_socket.close()
        self._bms_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MiRBatteryStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
