#!/usr/bin/env python3
import rclpy, random
from rclpy.node import Node
from functools import partial
from my_interfaces.srv import ComputeRectangleArea

class ComputeRectangleAreaClient(Node):
    def __init__(self):
        super().__init__('compute_rectangle_area_client')
        self.get_logger().info('Compute Rectangle Area Client has been started !!!')
        self.create_timer(3.0, self.timer_callback) # Cada 3 segundos se ejecuta el callback

    def timer_callback(self):
        length, width = float(random.randint(0, 9)), float(random.randint(0, 9))
        self.call_compute_rectangle_area_server(length, width)
        
    def call_compute_rectangle_area_server(self, length, width):
        client = self.create_client(ComputeRectangleArea, 'compute_rectangle_area') # Tipo de servicio, "nombre del servidor"
        while not client.wait_for_service(1.0):
            self.get_logger().info('Waiting for Server to start...')
        
        request = ComputeRectangleArea.Request()
        request.length, request.width = length, width
        future = client.call_async(request)

        def wait_for_response(future, length, width):
            response = future.result()
            print(response)
            print(f"{length} * {width} = {response.area}")

        future.add_done_callback(partial(wait_for_response, length=length, width=width))
        
def main(args=None):
    rclpy.init(args=args)
    node = ComputeRectangleAreaClient()
    try:
        rclpy.spin(node)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Terminated by the user !!!")

if __name__ == '__main__':
    main()