#!/usr/bin/env python3 
import rclpy, random
from rclpy.node import Node
from std_msgs.msg import Float32

class MyClassNode(Node):
 def __init__(self):
  super().__init__("random_generator")
  self.get_logger().info("Random number generator started...")
  self.create_timer(0.01, self.my_callback)
  self.pub = self.create_publisher(Float32, "random_number", 1)
  
 def my_callback(self):
  msg = Float32()
  msg.data = random.uniform(-10, 10)
  self.pub.publish(msg)
  
def main (args = None):
 rclpy.init(args = args)
 nodeh = MyClassNode()
 try: rclpy.spin(nodeh)
 except Exception as error: print (error)
 except KeyboardInterrupt: print("Node terminated by user!")
 
if __name__ == "__main__":
 main()




