import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MeshVisualizer(Node):
    def __init__(self):
        super().__init__('mesh_visualizer')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        timer_period = 0.5  # pubblica ogni 0.5 secondi
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "camera"  # Frame di riferimento
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "PD_namespace"
        marker.id = 3
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        # Specifica il percorso al file STL
        marker.mesh_resource = "package://my_robot_controller/meshes/payload_dispenser.stl"
        
        # Imposta la posizione del modello
        marker.pose.position.x = -0.19
        marker.pose.position.y = -0.19
        marker.pose.position.z = 0.00
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Scala del modello
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001

        # Colore del modello
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Pubblica il marker
        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MeshVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

