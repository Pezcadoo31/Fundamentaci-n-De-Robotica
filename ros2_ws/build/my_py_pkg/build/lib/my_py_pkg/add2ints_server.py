#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntegers(Node):
    def __init__(self):
        super().__init__('add_two_integers_server')
        self.get_logger().info('Add-Two-Integers Server has been started !!!')
        self.server = self.create_service(AddTwoInts, 'add_two_ints', self.service_callback) # TIpo de servicio, "nombre", callback

    def service_callback(self, request, response):
        response.sum = request.a + request.b
        print(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
        return response
    
def main(args=None):
    rclpy.init(args=args)
    nodeh = AddTwoIntegers()
    try : rclpy.spin(nodeh)
    except Exception as error : print(error)
    except KeyboardInterrupt : print("Terminated by the user !!!")

if __name__ == '__main__':
    main()