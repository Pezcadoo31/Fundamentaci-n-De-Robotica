#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node

class MyClassNode(Node):
 def __init__(self):
  super().__init__("my_node")
  self.create_timer(0.5, self.my_callback)
  self.counter = 1
  
 def my_callback(self):
  self.get_logger().info("Hola mundo " + str(self.counter))
  self.counter += 1

def main (args = None):
 rclpy.init(args = args)
 nodeh = MyClassNode()
 try: rclpy.spin(nodeh)
 except Exception as error: print (error)
 except KeyboardInterrupt: print("Node terminated by user!")
 
if __name__ == "__main__":
 main()

