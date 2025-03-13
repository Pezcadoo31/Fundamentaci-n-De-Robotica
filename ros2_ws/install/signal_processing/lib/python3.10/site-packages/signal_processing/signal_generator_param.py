#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Float32
from math import sin, cos, pi
from scipy.signal import square, sawtooth

class SignalGeneratorNode(Node):
    def __init__(self):
        super().__init__("signal_generator") # Inicializar el nodo
        self.get_logger().info("Signal generator started...")
        
        # Declarar parámetros
        self.declare_parameter("amplitud", 1.0)
        self.declare_parameter("phase", 0.0)
        self.declare_parameter("offset", 0.0)
        self.declare_parameter("periodo", 1.0)
        self.declare_parameter("sampling_time", 0.05)
        self.declare_parameter("signal_type", 1)
        
        # Leer parámetros inmediatamente después de declararlos
        self.read_params_callback()

        # Configurar variables iniciales
        self.start_time = time.time()  # Tiempo inicial
        self.pub = self.create_publisher(Float32, "signal_generator", 1) 
        self.create_timer(self.sampling_time, self.my_callback)  # Usar el tiempo de muestreo inicial
        self.create_timer(3.0, self.read_params_callback)  # Leer parámetros cada 3 segundos

    def read_params_callback(self):
        """Leer y actualizar los parámetros del nodo."""
        self.amplitude = self.get_parameter("amplitud").value
        self.phase = self.get_parameter("phase").value
        self.offset = self.get_parameter("offset").value
        self.periodo = self.get_parameter("periodo").value
        self.sampling_time = self.get_parameter("sampling_time").value
        self.signal_type = self.get_parameter("signal_type").value
        self.get_logger().info(f"Parameters updated: Amplitude={self.amplitude}, Phase={self.phase}, Offset={self.offset}, Periodo={self.periodo}, Sampling Time={self.sampling_time}, Signal Type={self.signal_type}")

    def my_callback(self):
        """Generar y publicar la señal."""
        current_time = time.time() - self.start_time
        omega = 2 * pi / self.periodo

        # Selección del tipo de señal
        if self.signal_type == 1:  # Sin
            signal_value = self.amplitude * sin(omega * current_time + self.phase) + self.offset
        elif self.signal_type == 2:  # Cos
            signal_value = self.amplitude * cos(omega * current_time + self.phase) + self.offset
        elif self.signal_type == 3:  # Square
            signal_value = self.amplitude * square(omega * current_time + self.phase) + self.offset
        elif self.signal_type == 4:  # Triangle
            signal_value = self.amplitude * sawtooth(omega * current_time + self.phase, 0.5) + self.offset
        elif self.signal_type == 5:  # Diente de sierra creciente
            signal_value = self.amplitude * sawtooth(omega * current_time + self.phase) + self.offset
        else:  # Diente de sierra decreciente
            signal_value = self.amplitude * sawtooth(omega * current_time + self.phase, 1) + self.offset

        # Publicar señal
        msg = Float32()
        msg.data = signal_value
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {signal_value}")


def main(args=None):
    rclpy.init(args=args)
    node = SignalGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node terminated by user!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
