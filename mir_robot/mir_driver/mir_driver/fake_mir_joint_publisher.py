#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_system_default


class FakeMirJointPublisher(Node):
    def __init__(self):
        super().__init__('fake_mir_joint_publisher')

        self.declare_parameter('tf_prefix', '')
        tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value.strip('/')
        if tf_prefix != "":
            tf_prefix = tf_prefix + '/'

        self.publisher_ = self.create_publisher(
            msg_type=JointState,
            topic='joint_states',
            qos_profile=qos_profile_system_default,
        )

        self.joint_names = [
            tf_prefix + 'left_wheel_joint',
            tf_prefix + 'right_wheel_joint',
            tf_prefix + 'fl_caster_rotation_joint',
            tf_prefix + 'fl_caster_wheel_joint',
            tf_prefix + 'fr_caster_rotation_joint',
            tf_prefix + 'fr_caster_wheel_joint',
            tf_prefix + 'bl_caster_rotation_joint',
            tf_prefix + 'bl_caster_wheel_joint',
            tf_prefix + 'br_caster_rotation_joint',
            tf_prefix + 'br_caster_wheel_joint',
        ]

        # Timer: alle 1 Sekunde publishen
        self.timer = self.create_timer(1.0, self.publish_joint_states)

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [0.0 for _ in js.name]
        js.velocity = [0.0 for _ in js.name]
        js.effort = [0.0 for _ in js.name]
        self.publisher_.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = FakeMirJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
