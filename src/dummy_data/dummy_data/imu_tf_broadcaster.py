import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time

class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10
        )
        self.imu_frame = 'imu_dynamic' # Child frame
        self.parent_frame = 'base_link' # Parent frame

    def imu_callback(self, msg: Imu):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.imu_frame

        # No translation — assume IMU is co-located with imu_link
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Use IMU orientation directly
        t.transform.rotation = msg.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()