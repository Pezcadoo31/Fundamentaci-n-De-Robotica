# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from custom_interfaces import SetProcessBool

#Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        # Retrieve sine wave parameters
        self.amplitude = 2.0
        self.omega  = 1.0

        #Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'motor_input_u', 10)
        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)
        
        #Create a messages and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        self.system_running = False

        self.cli = self.create_client(SetProcessBool, 'EnableProcess')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.send_request(True)

        self.get_logger().info("SetPoint Node Started \U0001F680")

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):

        if not self.system_running:
            return

        #Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        # Generate sine wave signal
        self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        # Publish the signal
        self.signal_publisher.publish(self.signal_msg)

    def send_request(self, enable=bool):
        request = SetProcessBool.Request()
        request.enable = enable
        
        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.system_running = True
                self.get_logger().info('Success: {0}'.format(response.success))
            else:
                self.system_running = False
                self.get_logger().info('Failure: {0}'.format(response.success))
        except Exception as e:
            self.simulation_running = False
            self.get_logger().info('Service call failed %r' % (e,))
            self.get_logger().info('Simulation Stopped')

#Main
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
