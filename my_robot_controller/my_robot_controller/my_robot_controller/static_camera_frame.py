import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticCameraFramePublisher(Node):
    def __init__(self):
        super().__init__('static_camera_frame_publisher')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def publish_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'world'  # Cambia a frame di riferimento appropriato
        static_transform.child_frame_id = 'camera'  # Nome del frame della camera

        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.4  # 40 cm

        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info('Static transform for camera published.')

def main(args=None):
    rclpy.init(args=args)
    node = StaticCameraFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
