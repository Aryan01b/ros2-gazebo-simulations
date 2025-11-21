#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_base_tf')

        self.declare_parameter('odom_topic', '/odom')
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = 'odom'   # Hardcode — matches your URDF/plugin
        self.base_frame = 'base_link'

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.get_logger().info(f'Subscribing to {self.odom_topic}')
        self.get_logger().info(f'Broadcasting TF: {self.odom_frame} -> {self.base_frame}')

    def odom_callback(self, msg):
        t = TransformStamped()
        
        # ✅ CRITICAL: Use odom message's timestamp, NOT current time
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()