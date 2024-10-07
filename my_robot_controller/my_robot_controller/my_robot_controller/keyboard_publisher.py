#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__("keyboard_publisher")
        self.keyboard_publisher = self.create_publisher(Int32, 'Keyboard_input', 10)
        self.get_logger().info("Keyboard publisher node has been started")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        input_number = int(input("Enter your number: "))
        msg = Int32()
        msg.data = input_number
        self.keyboard_publisher.publish(msg)
        self.get_logger().info("publishing " + str(msg.data))

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
