#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from math import sin, pi  

class SignalGeneratorNode(Node):
    def __init__(self):
        super().__init__("signal_generator")
        self.get_logger().info("Signal generator started...")     
        self.amplitude = 1      # Amplitud (A)
        self.frequency = 1      # Frecuencia w
        self.phase = 0          # Fase 
        self.offset = 0         # Offset (k)
        self.start_time = self.get_clock().now().nanoseconds / 1e9  
        self.signal_pub = self.create_publisher(Float32, "/signal", 1)     
        self.time_pub = self.create_publisher(Float32, "/time", 1)         
        self.create_timer(0.01, self.my_callback)                            

    def my_callback(self):
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time  
        omega = 2 * pi * self.frequency  # w = 2πf
        signal = self.amplitude * sin(omega * current_time + self.phase) + self.offset
        
        # Publicar la señal en /signal
        signal_msg = Float32()
        signal_msg.data = signal  
        self.signal_pub.publish(signal_msg) 
        
        # Publicar el tiempo en /time
        time_msg = Float32()
        time_msg.data = float(current_time)
        self.time_pub.publish(time_msg)
        
        # Loggear la señal y el tiempo
        self.get_logger().info(f"Time: {current_time}, Signal: {signal}")


def main(args=None):
    rclpy.init(args=args)
    node = SignalGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node terminated by user!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

