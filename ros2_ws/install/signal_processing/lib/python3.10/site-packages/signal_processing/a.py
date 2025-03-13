#!/usr/bin/env python3
import rclpy, time, sys
from rclpy.node import Node
from std_msgs.msg import Float32
from math import sin, cos, pi  
from scipy.signal import square, sawtooth

class SignalGeneratorNode(Node):
    def __init__(self, var):
        super().__init__("signal_generator")
        self.get_logger().info("Signal generator started...")
        self.amplitude = float(var[1])  # Amplitud (A)
        self.frequency = float(var[2])  # Frecuencia (f)
        self.phase = 0  # Fase inicial
        self.offset = float(var[3])  # Offset (k)
        self.periodo = float(var[4])  # Periodo de la señal
        self.sampling_time = float(var[5])  # Tiempo de muestreo
        self.signal_type = int(var[6])  # Tipo de señal (1-6)
        self.start_time = time.time()  # Tiempo inicial
        self.pub = self.create_publisher(Float32, "signal_generator", 1)
        self.create_timer(self.sampling_time, self.my_callback)

    def my_callback(self):
        current_time = time.time() - self.start_time  # Tiempo actual
        omega = 2 * pi / self.periodo  # w = 2π/p

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

        msg = Float32()
        msg.data = signal_value
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {signal_value}")


def main(args=None):
    if len(sys.argv) < 7:
        print("Error: Faltan argumentos")
        print("Proporcione: Amplitud, Frecuencia, Offset, Periodo, Tiempo de muestreo, Tipo de señal")
        return
    rclpy.init(args=args)
    node = SignalGeneratorNode(sys.argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node terminated by user!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

