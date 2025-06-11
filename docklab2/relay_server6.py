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
        super().__init__('relay_server6')
        # Set up GPIO
        # Setting up relays
        self.Relay_Ch1 = DigitalOutputDevice("BCM5")
        self.Relay_Ch2 = DigitalOutputDevice("BCM6")
        self.Relay_Ch3 = DigitalOutputDevice("BCM13")
        self.Relay_Ch4 = DigitalOutputDevice("BCM19")
        self.Relay_Ch5 = DigitalOutputDevice("BCM26")
        self.Relay_Ch6 = DigitalOutputDevice("BCM21")
        
        
        print("This message was written from initiation of relay_server_6 node!")
        
        # Add set Ch1 service
        self.srv = self.create_service(SetBool, 'set_Ch1', self.set_Ch1_callback)
        # Add set Ch2 service
        self.srv = self.create_service(SetBool, 'set_Ch2', self.set_Ch2_callback)
        # Add set Ch3 service
        self.srv = self.create_service(SetBool, 'set_Ch3', self.set_Ch3_callback)
        # Add set Ch4 service
        self.srv = self.create_service(SetBool, 'set_Ch4', self.set_Ch4_callback)
        # Add set Ch5 service
        self.srv = self.create_service(SetBool, 'set_Ch5', self.set_Ch5_callback)
        # Add set Ch6 service
        self.srv = self.create_service(SetBool, 'set_Ch6', self.set_Ch6_callback)

    def set_Ch1_callback(self, request, response):
        # Setting Ch1 to bool value from incoming request (True: on, False: off)
        self.Relay_Ch1.value = request.data 
        # Composing response
        response.success = True
        #self.get_logger().info('Incoming request\ndata: %d' % (request.data))

        return response
    
    def set_Ch2_callback(self, request, response):
        # Setting Ch2 to bool value from incoming request (True: on, False: off)
        self.Relay_Ch2.value = request.data 
        # Composing response
        response.success = True
        response.message = 'Changed Ch2 Channel'
        #self.get_logger().info('Incoming request\ndata: %d' % (request.data))

        return response
        
    def set_Ch3_callback(self, request, response):
        # Setting Ch3 to bool value from incoming request (True: on, False: off)
        self.Relay_Ch3.value = request.data 
        # Composing response
        response.success = True
        response.message = 'Changed Ch3 Channel'
        #self.get_logger().info('Incoming request\ndata: %d' % (request.data))

        return response
        
    def set_Ch4_callback(self, request, response):
        # Setting Ch4 to bool value from incoming request (True: on, False: off)
        self.Relay_Ch4.value = request.data 
        # Composing response
        response.success = True
        response.message = 'Changed Ch4 Channel'
        #self.get_logger().info('Incoming request\ndata: %d' % (request.data))

        return response
        
    def set_Ch5_callback(self, request, response):
        # Setting Ch5 to bool value from incoming request (True: on, False: off)
        self.Relay_Ch5.value = request.data 
        # Composing response
        response.success = True
        response.message = 'Changed Ch5 Channel'
        #self.get_logger().info('Incoming request\ndata: %d' % (request.data))

        return response

    def set_Ch6_callback(self, request, response):
        # Setting Ch6 to bool value from incoming request (True: on, False: off)
        self.Relay_Ch6.value = request.data 
        # Composing response
        response.success = True
        response.message = 'Changed Ch6 Channel'
        #self.get_logger().info('Incoming request\ndata: %d' % (request.data))

        return response

def main():
    rclpy.init()

    relay_server6 = RelayServer()

    rclpy.spin(relay_server6)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
