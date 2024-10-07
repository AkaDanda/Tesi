import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, Pose
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
import tf2_ros

class TFtoXYZNode(Node):
    def __init__(self):
        super().__init__('tf_to_xyz_node')
        self.publisher_marker = self.create_publisher(Marker, 'visualization_marker', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Sottoscrizione alla posizione (dal topic 'tf')
        self.subscription_tf = self.create_subscription(
            TFMessage,
            'tf',
            self.listener_callback_tf,
            10)

        # Sottoscrizione all'orientazione (dal topic 'pose_communication')
        self.subscription_pose = self.create_subscription(
            Pose,
            'pose_communication',
            self.listener_callback_pose,
            10)

        self.current_position = Point()
        self.current_orientation = Quaternion()  # Viene aggiornato solo una volta
        self.orientation_received = False  # Per tracciare se l'orientazione è stata ricevuta
        
        self.get_logger().info('TF to XYZ Node has been started.')

    def listener_callback_tf(self, msg):
        # Aggiorna la posizione dalla trasformazione TF
        for transform in msg.transforms:
            self.current_position.x = transform.transform.translation.x
            self.current_position.y = transform.transform.translation.y
            self.current_position.z = transform.transform.translation.z

            # Pubblica il marker per la visualizzazione della sfera
            marker_msg = Marker()
            marker_msg.header.frame_id = 'camera'
            marker_msg.header.stamp = self.get_clock().now().to_msg()
            marker_msg.ns = 'tf_markers'
            marker_msg.id = 0
            marker_msg.type = Marker.SPHERE
            marker_msg.action = Marker.ADD
            marker_msg.pose.position.x = self.current_position.x
            marker_msg.pose.position.y = self.current_position.y
            marker_msg.pose.position.z = self.current_position.z 
            marker_msg.pose.orientation.w = 1.0
            marker_msg.scale.x = 0.12
            marker_msg.scale.y = 0.12
            marker_msg.scale.z = 0.12
            marker_msg.color.r = 1.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 0.0
            marker_msg.color.a = 1.0
            self.publisher_marker.publish(marker_msg)

            # Pubblica il frame TF per il marker con rotazione fissa fino a quando non si riceve l'orientazione reale
            self.publish_transform(transform)

    def listener_callback_pose(self, msg):
        # Aggiorna l'orientazione dal messaggio Pose
        self.current_orientation = msg.orientation

        # Una volta ricevuta l'orientazione, la variabile viene impostata
        self.orientation_received = True
        self.get_logger().info(f'Received Orientation: {self.current_orientation}')

    def publish_transform(self, transform):
        # Pubblica la trasformazione con posizione e orientazione combinate o rotazione fissa
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera' # Il frame di riferimento genitore
        t.child_frame_id = 'payload_frame'  # Il frame associato al marker

        # Imposta la traduzione basata sulla posizione attuale
        t.transform.translation.x = self.current_position.x
        t.transform.translation.y = self.current_position.y
        t.transform.translation.z = self.current_position.z  
        

        

        # Imposta la rotazione: se l'orientazione è stata ricevuta, usa quella, altrimenti usa la rotazione fissa
        if self.orientation_received:
            t.transform.rotation = self.current_orientation
        else:
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0  # Quaternione identità (rotazione predefinita)
        

        # Pubblica la trasformazione tramite il TF broadcaster
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Published Transform with position: ({self.current_position.x}, {self.current_position.y}, {self.current_position.z - 40})')

def main(args=None):
    rclpy.init(args=args)
    node = TFtoXYZNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
