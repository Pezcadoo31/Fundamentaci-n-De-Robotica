#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

class MyClassNode(Node):
 def __init__(self):
  super().__init__("my_talker")
  self.create_timer(0.5, self.my_callback)
  self.pub = self.create_publisher(String, "my_chatter", 1)
  self.counter = 0
  
 def my_callback(self):
  self.counter += 1
  msg = String()
  msg.data = "Hola mundo " + str(self.counter)
  self.get_logger().info(msg.data)
  self.pub.publish(msg)
  
def main (args = None):
 rclpy.init(args = args)
 nodeh = MyClassNode()
 try: rclpy.spin(nodeh)
 except Exception as error: print (error)
 except KeyboardInterrupt: print("Node terminated by user!")
 
if __name__ == "__main__":
 main()

