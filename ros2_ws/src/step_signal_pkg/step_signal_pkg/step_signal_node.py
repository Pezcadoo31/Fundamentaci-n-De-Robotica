#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class StepSignalNode(Node):
    def __init__(self):
        super().__init__("step_signal_node")
        self.get_logger().info("Step Signal Node Started...")
        
        # Crear publicador en el tópico 'motor_input'
        self.publisher = self.create_publisher(Float32, 'motor_input', 10)

        # Parámetros de la señal escalón
        self.step_time = 15.0  # Tiempo en segundos donde ocurre el escalón
        self.amplitude = 1.0  # Valor de la señal después del escalón (escalón unitario)
        self.initial_value = 0.0  # Valor inicial de la señal

        # Obtener tiempo inicial
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Crear temporizador para publicar la señal cada 0.1s
        self.create_timer(0.1, self.publish_step_signal)

    def publish_step_signal(self):
        # Obtener el tiempo actual
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time

        # Generar señal escalón
        if elapsed_time < self.step_time:
            step_value = self.initial_value  # Valor inicial (0)
        else:
            step_value = self.amplitude  # Valor después del escalón (1)

        # Publicar el valor en el tópico
        msg = Float32()
        msg.data = step_value
        self.publisher.publish(msg)

        # Loggear el valor publicado
        self.get_logger().info(f"Published: {step_value}")

def main(args=None):
    rclpy.init(args=args)
    node = StepSignalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node terminated by user!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


