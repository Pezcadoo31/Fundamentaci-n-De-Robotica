#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

class MyClassNode(Node):
    def __init__(self):
        super().__init__("my_listener")
        self.sub = self.create_subscription(String, "my_chatter", self.my_callback, 1)
  
    def my_callback(self, msg):
        self.get_logger().info("Escuche = " + msg.data)
  
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


