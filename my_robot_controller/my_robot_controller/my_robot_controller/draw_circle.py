#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist #geometry_msgs va aggiunto nelle dipendenze di package.xml

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub= self.create_publisher(Twist, "/turtle1/cmd_vel", 10) #we create the publisher #HERE WE CREATE THE PUBLISHER
        self.timer_ = self.create_timer(0.5, self.send_velocity_command) #timer calls the send_velocity_function each 0.5 s
        self.get_logger().info("Draw circle node has been started")
    
    def send_velocity_command(self):
        msg = Twist() #we create a message object from the class Twist
        msg.linear.x=2.0
        msg.angular.z=1.0
        self.cmd_vel_pub.publish(msg) #we use the publish method of the pubblisher tu publish the message #HERE WE USE THE PUBLISHER
         

def main(args=None):
    rclpy.init(args=args)

    node = DrawCircleNode()

    rclpy.spin(node)
    rclpy.shutdown()