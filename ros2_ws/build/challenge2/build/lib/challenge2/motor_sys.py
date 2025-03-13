#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

class DCMotor(Node):
    def __init__(self):
        super().__init__('motor_sys')

        # Declarar parámetros
        self.declare_parameter('sample_time', 0.01)
        self.declare_parameter('sys_gain_K', 1.0)
        self.declare_parameter('sys_tau_T', 1.0)
        self.declare_parameter('initial_conditions', 0.0)

        # Obtener valores de los parámetros
        self.sample_time = self.get_parameter('sample_time').value
        self.param_K = self.get_parameter('sys_gain_K').value
        self.param_T = self.get_parameter('sys_tau_T').value
        self.initial_conditions = self.get_parameter('initial_conditions').value

        # Variables del motor
        self.input_u = 0.0
        self.output_y = self.initial_conditions

        # Publicador y suscriptor
        self.motor_input_sub = self.create_subscription(Float32, 'motor_input_u', self.input_callback, 10)
        self.motor_speed_pub = self.create_publisher(Float32, 'motor_output_y', 10)

        # Temporizador para la simulación del motor
        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        # Callback para actualizar parámetros en tiempo de ejecución
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info('Dynamical System Node Started 🚀')

    def timer_cb(self):
        """Simulación del motor DC."""
        self.output_y += (-1.0 / self.param_T * self.output_y + self.param_K / self.param_T * self.input_u) * self.sample_time
        self.output_y = np.clip(self.output_y, -10.0, 10.0)

        # Publicar la salida del motor
        msg = Float32()
        msg.data = self.output_y
        self.motor_speed_pub.publish(msg)

    def input_callback(self, input_sgn):
        """Callback para la entrada de control."""
        self.input_u = input_sgn.data

    def parameter_callback(self, params):
        """Callback para actualizar parámetros en tiempo real."""
        for param in params:
            if param.name == "sys_gain_K":
                if param.value < 0.0:
                    self.get_logger().warn("Invalid sys_gain_K! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="sys_gain_K cannot be negative")
                else:
                    self.param_K = param.value
                    self.get_logger().info(f"sys_gain_K updated to {self.param_K}")

            if param.name == "sys_tau_T":
                if param.value < 0.0:
                    self.get_logger().warn("Invalid sys_tau_T! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="sys_tau_T cannot be negative")
                else:
                    self.param_T = param.value
                    self.get_logger().info(f"sys_tau_T updated to {self.param_T}")

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = DCMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Motor_sys node terminated by user!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    