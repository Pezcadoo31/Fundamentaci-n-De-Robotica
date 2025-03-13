#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import math

class SignalGenerator(Node):
    def __init__(self):
        super().__init__("signal_generator")
        self.get_logger().info("Signal Generator Node has started.")

        # Publishers for signal and time
        self.signal_pub = self.create_publisher(Float32, "signal", 1)
        self.time_pub = self.create_publisher(Float32, "time", 1)

        # Timer initialization
        self.t0 = time.time()
        self.create_timer(0.1, self.timer_callback)  # Publish at 10 Hz

    def timer_callback(self):
        # Calculate time and sine wave
        t = time.time() - self.t0
        signal = math.sin(t)

        # Publish signal and time
        self.signal_pub.publish(Float32(data=signal))
        self.time_pub.publish(Float32(data=t))

        # Log the published values
        self.get_logger().info(f"Time: {t:.2f} s, Signal: {signal:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = SignalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node terminated by user!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
