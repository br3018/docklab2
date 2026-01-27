#!/usr/bin/env python

# Importing ROS2 Python Client Libraries 
import rclpy
from rclpy.node import Node

# Import service type 
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool

# Import message type 
from std_msgs.msg import String

# Import Python libraries
import time
import subprocess 

class BaseNode(Node):
    
    def __init__(self):

        super().__init__('base_node')
        # Setting up ABP1 interfaces 
        self.abp1_GRASP_pub_ = self.create_publisher(String, 'GRASP_flags', 10) #QoS arbitrarily set at 10
        self.abp1_GRASP_pub_ # prevent unused variable error
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Get user input
        print('''Command list: 
              "HOME":       Home GRAPPLE mechanism
              "OPEN":       Open end effectors
              "DOCK":       Run full docking procedure
              "SOFTDOCK":   Run only till soft dock
              "HARDDOCK":   Continue running till hard dock
              "DOCK":       Run docking in one smooth move
              "UNDOCK":     Run undocking procedure
              "STOP":       Stop mechanism movement
              ''')
        user_input = input('Enter command: ')

        match user_input:

            case 'HOME':
                self.abp1_GRASP_pub_.publish(String(data='GO_HOME'))
                time.sleep(0.25)

            case 'OPEN':
                self.abp1_GRASP_pub_.publish(String(data='GO_OPEN'))
                time.sleep(0.25)

            case 'SOFTDOCK':
                self.abp1_GRASP_pub_.publish(String(data='GO_SD'))
                time.sleep(0.25)

            case 'HARDDOCK':
                self.abp1_GRASP_pub_.publish(String(data='GO_HD'))
                time.sleep(0.25)

            case 'DOCK':
                self.abp1_GRASP_pub_.publish(String(data='GO_CAPTURE'))
                time.sleep(0.25)

            case 'UNDOCK':
                self.abp1_GRASP_pub_.publish(String(data='GO_RELEASE'))
                time.sleep(0.25)

            case 'STOP' | 'S' | 's' | 'stop' :
                self.abp1_GRASP_pub_.publish(String(data='STOP'))
                time.sleep(0.25)

            case _:
                print('Invalid command')

def main():
    rclpy.init()
    base_node = BaseNode()
    rclpy.spin(base_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
executable='/bin/bash'