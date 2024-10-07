#!/usr/bin/env python3 
import rclpy #importiamo la libreria di python per ROS2
from rclpy.node import Node
#the node must be created as a class outside the main as ROS2 uses Object Oriented Programming
class MyNode(Node): #il mio nodo eredita dalla classe Node della libreria rclpy.node
     def __init__(self):
          super().__init__("first_node")
          self.get_logger().info("Hello from ROS")
          self.create_timer(1.0, self.timer_callback) #qui chiamiamo la callback del timer igni secondo
          self.counter_ = 0 #creiamo un attributo inizializzato a 0

     def timer_callback(self): #questa è la callback del timer
          self.get_logger().info("Hello " + str(self.counter_)) 
          self.counter_+=1
          
     

def main(args=None): #Definiamo la funzione principale
     #inizializziamo la comunicazione con ROS2
    rclpy.init(args=args)#the args parameter of the init function is queal to the args of the main
    
    node = MyNode() #stiamo creando il nodo
    
    
    rclpy.spin(node) #per tenere il nodo vivo
    rclpy.shutdown()

if __name__== '__main__':
    main()