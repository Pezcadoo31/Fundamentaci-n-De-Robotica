#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from time import sleep
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('talker_node')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_cb)
        self.i = 0
    
    def timer_cb(self):
        msg = String()
        msg.data = f'Hello world {self.i}'  # Incluye el contador en el mensaje
        self. publisher. publish(msg)
        # Log del mensaje publicado
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        # Incrementa el contador
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node = MinimalSubscriber()  # Instancia única del nodo

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()  # Destrucción del nodo correctamente


if __name__ == '__main__':
    main()




