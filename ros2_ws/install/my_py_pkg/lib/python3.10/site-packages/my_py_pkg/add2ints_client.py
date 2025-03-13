#!/usr/bin/env python3
import rclpy, random
from rclpy.node import Node
from functools import partial
from example_interfaces.srv import AddTwoInts

class AddTwoIntegers(Node):
    def __init__(self):
        super().__init__('add_two_integers_client')
        self.get_logger().info('Add-Two-Integers Client has been started !!!')
        self.create_timer(3.0, self.timer_callback) # Cada 3 segundos se ejecuta el callback

    def timer_callback(self):
        a, b = random.randint(0, 9), random.randint(0, 9)
        self.call_add_two_ints_server(a, b)
        

    def call_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, 'add_two_ints') # Tipo de servicio, "nombre del servidor"
        while not client.wait_for_service(1.0):
            self.get_logger().info('Waiting for Server to start...')
        
        request = AddTwoInts.Request()
        request.a, request.b = a, b
        future = client.call_async(request)

        def wait_for_response(future, a, b):
            response = future.result()
            print(response)
            print(str(a) + " + " + str(b) + " = " + str(response.sum))

        future.add_done_callback(partial(wait_for_response, a=a, b=b))
        
    
def main(args=None):
    rclpy.init(args=args)
    nodeh = AddTwoIntegers()
    try : rclpy.spin(nodeh)
    except Exception as error : print(error)
    except KeyboardInterrupt : print("Terminated by the user !!!")

if __name__ == '__main__':
    main()