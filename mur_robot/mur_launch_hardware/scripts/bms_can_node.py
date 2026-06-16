#!/usr/bin/env python3
"""Read the MUR superstructure BMS over SocketCAN and publish battery state."""

from dataclasses import dataclass
import os
import subprocess

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


REQUEST_PRIORITY = 0x18
SOC_DATA_ID = 0x90


@dataclass
class BmsStatus:
    cumulative_total_voltage: float
    gathered_total_voltage: float
    current: float
    soc_percent: float


def parse_int_parameter(value):
    if isinstance(value, str):
        return int(value, 0)
    return int(value)


def parse_bool_parameter(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class BmsCanNode(Node):
    def __init__(self):
        super().__init__('bms_can_node')

        self.declare_parameter(
            'battery_node_id',
            '0x0240',
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 250000)
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('response_timeout', 0.8)
        self.declare_parameter('configure_can_interface', False)
        self.declare_parameter('soc_topic', 'bms_status/SOC')
        self.declare_parameter('battery_state_topic', 'battery_state')

        self.battery_node_id = parse_int_parameter(self.get_parameter('battery_node_id').value)
        self.can_interface = str(self.get_parameter('can_interface').value)
        self.can_bitrate = int(self.get_parameter('can_bitrate').value)
        self.response_timeout = float(self.get_parameter('response_timeout').value)
        publish_frequency = float(self.get_parameter('publish_frequency').value)

        if publish_frequency <= 0.0:
            raise ValueError('publish_frequency must be greater than zero')
        if self.response_timeout <= 0.0:
            raise ValueError('response_timeout must be greater than zero')

        self.bus = None
        self.can_module = None
        self.last_status = None

        self.soc_publisher = self.create_publisher(
            Float32, str(self.get_parameter('soc_topic').value), 10)
        self.battery_state_publisher = self.create_publisher(
            BatteryState, str(self.get_parameter('battery_state_topic').value), 10)

        if parse_bool_parameter(self.get_parameter('configure_can_interface').value):
            self.configure_can_interface()

        self.timer = self.create_timer(1.0 / publish_frequency, self.query_and_publish)
        self.get_logger().info(
            f'Reading BMS node 0x{self.battery_node_id:04x} on '
            f'{self.can_interface} at {self.can_bitrate} bit/s.'
        )

    def configure_can_interface(self):
        command = [
            'ip',
            'link',
            'set',
            self.can_interface,
            'up',
            'type',
            'can',
            'bitrate',
            str(self.can_bitrate),
        ]
        try:
            subprocess.run(command, check=True)
        except (OSError, subprocess.CalledProcessError) as exc:
            self.get_logger().warn(
                f'Failed to configure CAN interface {self.can_interface}: {exc}. '
                'Bring it up externally or run this node with sufficient privileges.',
                throttle_duration_sec=10.0,
            )

    def open_bus(self):
        if self.bus is not None:
            return self.bus

        if not os.path.exists(f'/sys/class/net/{self.can_interface}'):
            self.get_logger().warn(
                f'SocketCAN interface {self.can_interface} does not exist.',
                throttle_duration_sec=5.0,
            )
            return None

        try:
            import can
        except ImportError as exc:
            self.get_logger().error(
                f'python-can is not installed: {exc}. Install the python3-can dependency.')
            return None

        try:
            self.can_module = can
            self.bus = can.Bus(
                interface='socketcan',
                channel=self.can_interface,
                bitrate=self.can_bitrate,
            )
        except (can.CanError, OSError) as exc:
            self.get_logger().warn(
                f'Could not open SocketCAN bus {self.can_interface}: {exc}',
                throttle_duration_sec=5.0,
            )
            self.bus = None
        return self.bus

    def query_and_publish(self):
        status = self.query_status()
        if status is None:
            return

        self.last_status = status

        soc_msg = Float32()
        soc_msg.data = round(status.soc_percent, 1)
        self.soc_publisher.publish(soc_msg)

        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.voltage = status.gathered_total_voltage
        battery_msg.current = status.current
        battery_msg.percentage = max(0.0, min(1.0, status.soc_percent / 100.0))
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self.battery_state_publisher.publish(battery_msg)

        self.get_logger().debug(
            f'BMS SOC={status.soc_percent:.1f}%, '
            f'voltage={status.gathered_total_voltage:.1f} V, '
            f'current={status.current:.1f} A'
        )

    def query_status(self):
        bus = self.open_bus()
        if bus is None:
            return None

        request_id = (((REQUEST_PRIORITY << 8) | SOC_DATA_ID) << 16) | self.battery_node_id
        try:
            bus.send(self.can_module.Message(
                arbitration_id=request_id,
                data=[],
                is_extended_id=True,
            ))
        except self.can_module.CanError as exc:
            self.get_logger().warn(f'Failed to send BMS request: {exc}', throttle_duration_sec=5.0)
            self.close_bus()
            return None

        deadline = self.get_clock().now().nanoseconds * 1e-9 + self.response_timeout
        while rclpy.ok():
            remaining = deadline - self.get_clock().now().nanoseconds * 1e-9
            if remaining <= 0.0:
                self.get_logger().warn('Timed out waiting for BMS SOC response.',
                                       throttle_duration_sec=5.0)
                return None

            try:
                msg = bus.recv(timeout=remaining)
            except self.can_module.CanError as exc:
                self.get_logger().warn(f'Failed to read BMS response: {exc}',
                                       throttle_duration_sec=5.0)
                self.close_bus()
                return None

            if msg is None:
                continue
            if ((int(msg.arbitration_id) >> 16) & 0xff) != SOC_DATA_ID:
                continue
            return self.decode_soc_response(msg.data)

        return None

    def decode_soc_response(self, data):
        if len(data) < 8:
            self.get_logger().warn(
                f'Ignoring short BMS SOC response with {len(data)} bytes.',
                throttle_duration_sec=5.0,
            )
            return None

        return BmsStatus(
            cumulative_total_voltage=int.from_bytes(data[0:2], byteorder='big') / 10.0,
            gathered_total_voltage=int.from_bytes(data[2:4], byteorder='big') / 10.0,
            current=int.from_bytes(data[4:6], byteorder='big') / 10.0 - 3000.0,
            soc_percent=int.from_bytes(data[6:8], byteorder='big') / 10.0,
        )

    def close_bus(self):
        if self.bus is None:
            return
        try:
            self.bus.shutdown()
        except Exception:
            pass
        self.bus = None

    def destroy_node(self):
        self.close_bus()
        super().destroy_node()


def main():
    rclpy.init()
    node = BmsCanNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
