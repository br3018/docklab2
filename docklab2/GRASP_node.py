#!/usr/bin/env python

# Importing ROS2 Python Client Libraries 
import rclpy
from rclpy.node import Node

# Import service type 
from std_srvs.srv import Trigger

# Import message type 
from std_msgs.msg import String

# Import Python libraries 
import serial

'''
Functions: 
serial_connect: service 
 - Connects to Teensy 4.1 motor driver on prototype MDE board over serial comms
serial_disconnect: service
 - Disconnects from Teensy 4.1 motor driver on prototype MDE board over serial comms

'''

class GRASPNode(Node):
    # Constructor
    def __init__(self):
        # Set node name 
        super().__init__('GRASP_node')
        # Set up Serial
        self.ser = serial.Serial()
        self.ser.port = '/dev/ttyACM0'
        self.ser.baudrate = 9600
        self.ser.timeout = 1 #timeout of 1s
        
        # Add serial connect service 
        self.srv = self.create_service(Trigger, 'serial_connect', self.serial_connect_callback)
        # Add serial close service
        self.srv = self.create_service(Trigger, 'serial_disconnect', self.serial_disconnect_callback)
        
        # Add GRASP state subscriber
        self.subscription = self.create_subscription(String, 'GRASP_state', self.GRASP_state_callback, 10) #QoS arbitrarily set at 10
        self.subscription # prevent unused variable error

    def serial_connect_callback(self, request, response):
        # Opening serial port
        try:    
            self.ser.open()
        except:
            self.get_logger().warning('Serial communications error')
        # Composing response
        # Checking serial port is open
        if self.ser.is_open == True: 
            self.ser.reset_input_buffer()
            response.success = True
            self.get_logger().info('Established connection with MDE')
        else:
            response.success = False
            self.get_logger().warning('Failed to establish connection with MDE')
        return response
        
    def serial_disconnect_callback(self, request, response):
        # Closing serial port
        try:    
            self.ser.close()
        except:
            self.get_logger().warning('Serial communications error')
        # Composing response
        # Checking serial port is closed
        if self.ser.is_open == False: 
            response.success = True
            self.get_logger().info('Disconnected from MDE')
        else:
            response.success = False
            self.get_logger().warning('Failed to disconnect from MDE')
        return response
        
    def GRASP_state_callback(self, msg):
        # Checking GRASP state and assigning number
        match msg.data:
            case "HOME":
                state = "1"
                self.get_logger().info('Updating GRASP state to HOME')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case "OPEN":
                state = "2"
                self.get_logger().info('Updating GRASP state to OPEN')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case "CLOSE":
                state = "3"
                self.get_logger().info('Updating GRASP state to CLOSE')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case "RETRACT":
                state = "4"
                self.get_logger().info('Updating GRASP state to RETRACT')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case "RELEASE":
                state = "5"
                self.get_logger().info('Updating GRASP state to RELEASE')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case "RUN":
                state = "6"
                self.get_logger().info('Updating GRASP state to RUN')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case "UNDOCK":
                state = "7"
                self.get_logger().info('Updating GRASP state to UNDOCK')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case "MORE":
                state = "8"
                self.get_logger().info('Updating GRASP state to MORE')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case "LESS":
                state = "9"
                self.get_logger().info('Updating GRASP state to LESS')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case "AVC_HOME":
                state = "10"
                self.get_logger().info('Updating GRASP state to AVC_HOME')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case "AVC_RETRACT":
                state = "12"
                self.get_logger().info('Updating GRASP state to AVC_RETRACT')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case "AVC_MORE":
                state = "13"
                self.get_logger().info('Updating GRASP state to AVC_MORE')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case "AVC_LESS":
                state = "14"
                self.get_logger().info('Updating GRASP state to AVC_LESS')
                self.ser.reset_input_buffer()
                self.ser.write(state.encode())
            case _: # Catch invalid command 
                self.get_logger().warning('Unknown GRASP state recieved')
                


def main():
    rclpy.init()

    GRASP_node = GRASPNode()

    rclpy.spin(GRASP_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
