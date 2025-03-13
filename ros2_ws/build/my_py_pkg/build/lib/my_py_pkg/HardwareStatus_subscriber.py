#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from my_interfaces.msg import HardwareStatus 

class MyClassNode(Node):
    def __init__(self):
        super().__init__("hardware_status_subscriber")
        self.sub = self.create_subscription(HardwareStatus, "status", self.my_callback, 1)
  
    def my_callback(self, msg):
        self.get_logger().info("Temperature = " + str(msg.temperature))
        self.get_logger().info("Are motors ready = " + str(msg.are_motors_ready))
        self.get_logger().info("Debug status = " + msg.debug_status)
        print("---")
  
def main(args=None):
    rclpy.init(args=args)
    nodeh = MyClassNode()
    try:
        rclpy.spin(nodeh)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Node terminated by user!")
 
if __name__ == "__main__":
    main()


