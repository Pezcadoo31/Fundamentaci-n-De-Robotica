#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult

class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('sp_gen')

        # Declaración de parámetros con descriptores
        self.declare_parameter('signal_type', 'sine', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Tipo de señal: 'sine', 'square' o 'triangle'"
        ))
        self.declare_parameter('amplitude', 1.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Amplitud de la señal"
        ))
        self.declare_parameter('frequency', 1.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Frecuencia de la señal en Hz"
        ))
        self.declare_parameter('offset', 0.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Desplazamiento de la señal"
        ))

        self.timer_period = 0.1  # Periodo de muestreo en segundos

        # Publicador
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        # Tiempo inicial
        self.start_time = self.get_clock().now()

        # Callback para actualizar parámetros en tiempo de ejecución
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("SetPoint Node Started 🚀")

    def parameter_callback(self, params):
        """Callback para actualizar parámetros en tiempo real."""
        for param in params:
            if param.name == "signal_type":
                self.get_logger().info(f"Signal type updated to {param.value}")
            elif param.name == "amplitude":
                self.get_logger().info(f"Amplitude updated to {param.value}")
            elif param.name == "frequency":
                self.get_logger().info(f"Frequency updated to {param.value}")
            elif param.name == "offset":
                self.get_logger().info(f"Offset updated to {param.value}")
        return SetParametersResult(successful=True)

    def timer_cb(self):
        """Callback del temporizador para generar y publicar la señal deseada."""
        # Obtener parámetros en tiempo real
        signal_type = self.get_parameter('signal_type').value
        amplitude = self.get_parameter('amplitude').value
        frequency = self.get_parameter('frequency').value
        offset = self.get_parameter('offset').value

        # Calcular tiempo transcurrido
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9  # Convertir a segundos
        omega = 2 * np.pi * frequency  # Convertir frecuencia a radianes

        # Generar la señal
        if signal_type == 'sine':
            signal_value = amplitude * np.sin(omega * elapsed_time) + offset
        elif signal_type == 'square':
            signal_value = amplitude * np.sign(np.sin(omega * elapsed_time)) + offset
        elif signal_type == 'triangle':
            signal_value = amplitude * (1 - 4 * np.abs(np.mod((elapsed_time * frequency), 1) - 0.5)) + offset
        else:
            signal_value = offset  # Señal constante

        # Publicar la señal generada
        msg = Float32()
        msg.data = signal_value
        self.signal_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SetPointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SetPoint node terminated by user!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
