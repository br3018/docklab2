#!/usr/bin/env python

# Importing ROS2 Python Client Libraries 
import rclpy
from rclpy.node import Node

# Import service type 
from std_srvs.srv import SetBool

# Import GPIO Libraries 
from gpiozero import DigitalOutputDevice

class RelayServer(Node):
    # Constructor
    def __init__(self):
        # Set node name 
        super().__init__('relay_server')
        # Set up GPIO
        # Setting up relays
        self.Relay_Ch1 = DigitalOutputDevice("BCM26")
        self.Relay_Ch2 = DigitalOutputDevice("BCM20")
        self.Relay_Ch3 = DigitalOutputDevice("BCM21")
        
        # Add set Ch1 service
        self.srv = self.create_service(SetBool, 'set_Ch1', self.set_Ch1_callback)

    def set_Ch1_callback(self, request, response):
        # Setting Ch1 to bool value from incoming request (True: on, False: off)
        self.Relay_Ch1.value = request.data 
        # Composing response
        response.success = True
        response.message = 'Changed Ch1 Channel'
        #self.get_logger().info('Incoming request\ndata: %d' % (request.data))

        return response


def main():
    rclpy.init()

    relay_server = RelayServer()

    rclpy.spin(relay_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
