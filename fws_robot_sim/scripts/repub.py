#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped




class CmdVelRepublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_republisher')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(TwistStamped, '/mobile_base_controller/cmd_vel', 10)

    def cmd_vel_callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = msg
        self.publisher.publish(twist_stamped)
        print('Published TwistStamped message')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()