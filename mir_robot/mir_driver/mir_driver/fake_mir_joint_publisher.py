#!/usr/bin/env python3
# Copyright (c) 2018-2022, Martin Günther (DFKI GmbH) and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Martin Günther

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_system_default


class fake_mir_joint_publisher(Node):
    def __init__(self):
        super().__init__('fake_mir_joint_publisher')

        self.declare_parameter('tf_prefix', '')
        tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value.strip('/')
        if tf_prefix != "":
            tf_prefix = tf_prefix + '/'

        pub = self.create_publisher(
            msg_type=JointState,
            topic='joint_states',  # no prefix to joint states, just namespace
            qos_profile=qos_profile_system_default,  # TODO Check QoS Settings
        )

        pub_rate = 1

        while rclpy.ok():
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [
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
            js.position = [0.0 for _ in js.name]
            js.velocity = [0.0 for _ in js.name]
            js.effort = [0.0 for _ in js.name]
            pub.publish(js)
            time.sleep(pub_rate)


def main(args=None):
    rclpy.init(args=args)
    node = fake_mir_joint_publisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
