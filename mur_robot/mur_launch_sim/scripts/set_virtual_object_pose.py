#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VirtualObjectPoseSetter(Node):
    def __init__(self):
        super().__init__('set_virtual_object_pose')
        self.publisher_ = self.create_publisher(PoseStamped, '/virtual_object/set_pose', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_pose)

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 1.5
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info('Published virtual object pose')
        self.timer.cancel()  # only send once

def main(args=None):
    rclpy.init(args=args)
    node = VirtualObjectPoseSetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
