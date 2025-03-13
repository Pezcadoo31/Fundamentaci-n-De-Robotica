#!/usr/bin/env python3 
import rclpy, random
from rclpy.node import Node
from my_interfaces.msg import HardwareStatus
from random import uniform

class MyClassNode(Node):
 def __init__(self):
  super().__init__("hardware_status_publisher")
  self.create_timer(0.5, self.my_callback)
  self.pub = self.create_publisher(HardwareStatus, "status", 1)
  self.counter = 0
  
 def my_callback(self):
  self.counter += 1
  msg = HardwareStatus()
  msg.temperature = random.uniform(20, 40)
  msg.are_motors_ready = True if msg.temperature < 30 else False
  msg.debug_status = "Hola mundo " + str(self.counter)
  self.pub.publish(msg)
  
def main (args = None):
 rclpy.init(args = args)
 nodeh = MyClassNode()
 try: rclpy.spin(nodeh)
 except Exception as error: print (error)
 except KeyboardInterrupt: print("Node terminated by user!")
 
if __name__ == "__main__":
 main()

