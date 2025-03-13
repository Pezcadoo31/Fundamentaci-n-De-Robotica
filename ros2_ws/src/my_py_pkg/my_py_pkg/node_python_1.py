#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node

class MyClassNode(Node):
 def __init__(self):
  super().__init__("my_node")
  self.get_logger().info("Hola mundo")

def main (args = None):
 rclpy.init(args = args)
 nodeh = MyClassNode()
 rclpy.shutdown()
 
if __name__ == "__main__":
 main()

