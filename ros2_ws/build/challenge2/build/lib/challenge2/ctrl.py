#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
from rcl_interfaces.msg import SetParametersResult

class PIDController(Node):
    def __init__(self):
        super().__init__('ctrl')

        # Declarar parámetros con valores por defecto
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.0)
        self.declare_parameter('sample_time', 0.01)
        self.declare_parameter('integral_limit', 10.0)  # Limite para evitar windup

        # Obtener valores de los parámetros
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.Ts = self.get_parameter('sample_time').value
        self.integral_limit = self.get_parameter('integral_limit').value

        # Variables del PID
        self.set_point = 0.0
        self.motor_output = 0.0
        self.prev_error = 0.0
        self.integral = 0.0

        # Suscriptores
        self.create_subscription(Float32, 'set_point', self.set_point_callback, 1)
        self.create_subscription(Float32, 'motor_output_y', self.motor_output_callback, 1)

        # Publicador
        self.pub = self.create_publisher(Float32, 'motor_input_u', 1)

        # Temporizador para la ejecución periódica del control
        self.timer = self.create_timer(self.Ts, self.control_loop)

        # Callback para actualizar parámetros en tiempo de ejecución
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("PID Controller Node Started 🚀")

    def parameter_callback(self, params):
        """Callback para actualizar parámetros en tiempo real."""
        for param in params:
            if param.name == "Kp":
                self.Kp = max(0.0, param.value)
                self.get_logger().info(f"Kp updated to {self.Kp}")
            elif param.name == "Ki":
                self.Ki = max(0.0, param.value)
                self.get_logger().info(f"Ki updated to {self.Ki}")
            elif param.name == "Kd":
                self.Kd = max(0.0, param.value)
                self.get_logger().info(f"Kd updated to {self.Kd}")
            elif param.name == "sample_time":
                self.Ts = max(0.001, param.value)  # Mínimo 1 ms
                self.get_logger().info(f"Sample time updated to {self.Ts}")
            elif param.name == "integral_limit":
                self.integral_limit = max(0.0, param.value)
                self.get_logger().info(f"Integral limit updated to {self.integral_limit}")
        return SetParametersResult(successful=True)

    def set_point_callback(self, msg):
        """Actualiza el set point."""
        self.set_point = msg.data

    def motor_output_callback(self, msg):
        """Actualiza la salida del motor."""
        self.motor_output = msg.data

    def control_loop(self):
        """Bucle de control PID que se ejecuta en cada iteración del temporizador."""
        error = self.set_point - self.motor_output

        # Cálculo del control PID
        self.integral += error * self.Ts
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)  # Anti-windup
        derivative = (error - self.prev_error) / self.Ts
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Limitar la señal de control para evitar valores fuera de rango
        u = np.clip(u, -10.0, 10.0)

        # Publicar la salida del controlador
        msg = Float32()
        msg.data = u
        self.pub.publish(msg)

        # Actualizar error previo
        self.prev_error = error

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Controller node terminated by user!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
