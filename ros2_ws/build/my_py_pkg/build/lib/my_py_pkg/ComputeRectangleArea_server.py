#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces.srv import ComputeRectangleArea

class ComputeRectangleAreaServer(Node):
    def __init__(self):
        super().__init__('compute_rectangle_area_server')
        self.get_logger().info('Compute Rectangle Area Server has been started !!!')
        self.server = self.create_service(ComputeRectangleArea, 'compute_rectangle_area', self.service_callback) # Tipo de servicio, "nombre", callback

    def service_callback(self, request, response):
        response.area = request.length * request.width
        print(f"{request.length} * {request.width} = {response.area}")
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = ComputeRectangleAreaServer()
    try:
        rclpy.spin(node)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Terminated by the user !!!")

if __name__ == '__main__':
    main()