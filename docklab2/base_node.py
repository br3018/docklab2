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
    # Constructor
    def __init__(self):
        # Set node name 
        super().__init__('base_node')

        # Setting up ABP1 interfaces 
        self.abp1_GRASP_pub_ = self.create_publisher(String, 'abp1/GRASP_state', 10) #QoS arbitrarily set at 10
        self.abp1_GRASP_pub_ # prevent unused variable error
        # Setting up client for abp1 namespace services
        self.abp1_airb_cli_ = self.create_client(SetBool, 'abp1/set_Ch1')
        self.abp1_airb_req = SetBool.Request()
        self.abp1_GRASPsc_cli_ = self.create_client(Trigger, 'abp1/serial_connect')
        self.abp1_GRASPsc_req = Trigger.Request()
        self.abp1_GRASPsd_cli_ = self.create_client(Trigger, 'abp1/serial_disconnect')
        self.abp1_GRASPsd_req = Trigger.Request()

        # Setting up ABP2 interfaces
        # Setting up client for abp2 namespace services
        self.abp2_airb_cli_ = self.create_client(SetBool, 'abp2/set_Ch1')
        self.abp2_airb_req = SetBool.Request()

        # Setting up timer for getting user input 
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Get user input
        print('''Command list: 
              Switch on air bearings: "ON"
              Switch off air bearings: "OFF"
              Initialise GRASP: "INIT" (Connect MDE and home grapple and AVC)
              Run docking procedure: "DOCK" (Begin recording to a rosbag and run docking procedure)
              Run undocking procedure: "UNDOCK" (Begin recording to a rosbag and run undocking procedure)
              ''')
        user_input = input('Enter command: ')
        # Process user input
        match user_input:
            case 'ON':
                self.abp1_airb_req.data = True
                self.abp1_airb_cli_.call_async(self.abp1_airb_req)
                self.abp2_airb_req.data = True
                self.abp2_airb_cli_.call_async(self.abp2_airb_req)
            case 'OFF':
                self.abp1_airb_req.data = False
                self.abp1_airb_cli_.call_async(self.abp1_airb_req)
                self.abp2_airb_req.data = False
                self.abp2_airb_cli_.call_async(self.abp2_airb_req)
            case 'INIT':
                self.abp1_GRASPsc_cli_.call_async(self.abp1_GRASPsc_req)
                time.sleep(1) # Wait for serial connection to establish
                self.abp1_GRASP_pub_.publish(String(data='HOME'))
                time.sleep(20)
                self.abp1_GRASP_pub_.publish(String(data='AVC_HOME'))
                time.sleep(20)
            case 'DOCK':
                # Start recording to rosbag "docking" + date and time in YYYYMMDDHHMMSS format
                filename = 'docking_' + time.strftime('%Y%m%d%H%M%S')
                command = 'ros2 bag record --all --output' + filename
                self.bagprocess = subprocess.Popen([command], stdin=subprocess.PIPE, shell=True, cwd="/home/labpi/docklab2_ws/bag_files", executable='/bin/bash')

                # Run docking procedure
                # Start air bearings
                self.abp1_airb_req.data = True
                self.abp1_airb_cli_.call_async(self.abp1_airb_req)
                self.abp2_airb_req.data = True
                self.abp2_airb_cli_.call_async(self.abp2_airb_req)
                # Start GRASP
                self.abp1_GRASP_pub_.publish(String(data='RUN'))
                # Delay whilst docking procedure runs
                time.sleep(20)
                # Stop air bearings
                self.abp1_airb_req.data = False
                self.abp1_airb_cli_.call_async(self.abp1_airb_req)
                self.abp2_airb_req.data = False
                self.abp2_airb_cli_.call_async(self.abp2_airb_req)
                # Stop recording to rosbag
                self.bagprocess.kill()

            case 'UNDOCK':
                # Start recording to rosbag "docking" + date and time in YYYYMMDDHHMMSS format
                filename = 'undocking_' + time.strftime('%Y%m%d%H%M%S')
                command = 'ros2 bag record --all --output' + filename
                self.bagprocess = subprocess.Popen([command], stdin=subprocess.PIPE, shell=True, cwd="/home/labpi/docklab2_ws/bag_files", executable='/bin/bash')

                # Run undocking procedure
                # Start air bearings
                self.abp1_airb_req.data = True
                self.abp1_airb_cli_.call_async(self.abp1_airb_req)
                self.abp2_airb_req.data = True
                self.abp2_airb_cli_.call_async(self.abp2_airb_req)
                # Start GRASP
                self.abp1_GRASP_pub_.publish(String(data='UNDOCK'))
                # Delay whilst undocking procedure runs
                time.sleep(20)
                # Stop air bearings
                self.abp1_airb_req.data = False
                self.abp1_airb_cli_.call_async(self.abp1_airb_req)
                self.abp2_airb_req.data = False
                self.abp2_airb_cli_.call_async(self.abp2_airb_req)
                # Stop recording to rosbag
                self.bagprocess.kill()

            case default:
                print('Invalid command')




def main():
    rclpy.init()

    base_node = BaseNode()

    rclpy.spin(base_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
executable='/bin/bash'