#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ProcessedSignalNode(Node):
    def __init__(self):
        super().__init__("PROCESS")  # Inicializar el nodo
        self.get_logger().info("Signal processor has started...")
        
        # Declarar parámetros
        self.declare_parameter("amplitude_gain", 0.5)  # Ganancia de amplitud
        self.declare_parameter("offset", 0.0)  # Offset adicional
        
        # Inicializar variables
        self.u = 0.0  # Entrada (señal recibida)
        self.y = 0.0  # Salida (señal procesada)
        self.msg = Float32()  # Mensaje para publicar
        sampling_time = 0.05  # Tiempo de muestreo

        # Definir subscriptores, publicadores y timers
        self.signal_sub = self.create_subscription(Float32, "signal_generator", self.signal_callback, 1)
        self.proc_signal_pub = self.create_publisher(Float32, "processed_signal", 1)
        self.create_timer(sampling_time, self.processed_signal_callback)

    def signal_callback(self, msg):
        """Callback para recibir la señal original."""
        self.u = msg.data  # Leer señal entrante

    def processed_signal_callback(self):
        """Procesar y publicar la señal."""
        # Leer parámetros dinámicos
        amplitude_gain = self.get_parameter("amplitude_gain").value
        offset = self.get_parameter("offset").value

        # Procesar señal: aplicar ganancia y offset
        self.y = amplitude_gain * self.u + offset

        # Publicar la señal procesada
        self.msg.data = self.y
        self.proc_signal_pub.publish(self.msg)

        # Loggear la señal procesada
        self.get_logger().info(f"Processed Signal: {self.y}")

def main(args=None):
    rclpy.init(args=args)
    node = ProcessedSignalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node terminated by user!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
