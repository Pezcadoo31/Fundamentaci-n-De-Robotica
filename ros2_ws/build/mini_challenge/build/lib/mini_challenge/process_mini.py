#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from math import sin, pi

class ProcessNode(Node):
    def __init__(self):
        super().__init__("process")
        self.get_logger().info("Process node started...")

        # Parámetros de procesamiento
        self.offset_alpha = 2   # Offset para que la señal sea positiva ≥ 0
        self.phase_shift = pi/2 # Cambio de fase 
        
        # Suscriptores
        self.signal_sub = self.create_subscription(Float32, "/signal", self.signal_callback, 1)
        self.time_sub = self.create_subscription(Float32, "/time", self.time_callback, 1)
        
        # Publicador
        self.proc_signal_pub = self.create_publisher(Float32, "/proc_signal", 1)
        
        # Temporizador para publicar a 10 Hz (0.1 segundos)
        self.create_timer(0.1, self.timer_callback)  
        
        # Almacenar los datos recibidos
        self.signal_data = 0
        self.time_data = 0

    def signal_callback(self, msg):
        # Recibir la señal del tópico /signal
        self.signal_data = msg.data

    def time_callback(self, msg):
        # Recibir el tiempo del tópico /time
        self.time_data = msg.data

    def timer_callback(self):
        # Procesar la señal
        processed_signal = self.process_signal(self.signal_data)
        
        # Publicar la señal procesada
        proc_msg = Float32()
        proc_msg.data = processed_signal
        self.proc_signal_pub.publish(proc_msg)
        
        # Loggear la señal procesada
        self.get_logger().info(f"Processed Signal: {processed_signal}")

    def process_signal(self, signal):
        # Aplicar el offset para que la señal sea siempre positiva 
        signal_with_offset = signal + self.offset_alpha
        
        # Reducir la amplitud a la mitad
        signal_reduced_amplitude = signal_with_offset * 0.5
        
        # Aplicar el cambio de fase
        processed_signal = sin(signal_reduced_amplitude + self.phase_shift)
        
        return processed_signal

def main(args=None):
    rclpy.init(args=args)
    node = ProcessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node terminated by user!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

