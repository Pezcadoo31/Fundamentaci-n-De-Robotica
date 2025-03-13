#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class ProcessNode(Node):
    def __init__(self):
        super().__init__("process")
        self.get_logger().info("Process Node has started.")

        # Subscriptions to /signal and /time
        self.signal_sub = self.create_subscription(Float32, "signal", self.signal_callback, 10)
        self.time_sub = self.create_subscription(Float32, "time", self.time_callback, 10)

        # Publisher for processed signal
        self.proc_signal_pub = self.create_publisher(Float32, "proc_signal", 10)

        # Phase shift parameter (45 degrees = π/4 radians)
        self.phase_shift = 0

    def signal_callback(self, msg):
        # Process the signal: offset, scale amplitude, and add phase shift
        offset_signal = (msg.data + 1) * 0.5
        processed_signal_value = np.sin(offset_signal + self.phase_shift)

        # Publish the processed signal
        self.proc_signal_pub.publish(Float32(data=processed_signal_value))

        # Log the processed signal
        self.get_logger().info(f"Processed Signal: {processed_signal_value:.4f}")

    def time_callback(self, msg):
        # Optional: Log or use time if necessary
        self.get_logger().info(f"Received Time: {msg.data:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ProcessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node terminated by user!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
