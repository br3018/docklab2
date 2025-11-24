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
        self.abp1_GRASP_pub_ = self.create_publisher(String, 'GRASP_flags', 10) #QoS arbitrarily set at 10
        self.abp1_GRASP_pub_ # prevent unused variable error

        # Setting up timer for getting user input 
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Get user input
        print('''Command list: 
              "HOME":       Home GRAPPLE mechanism
              "OPEN":       Open end effectors
              "DOCK":       Run docking procedure
              "UNDOCK":     Run undocking procedure
              "STOP":       Stop mechanism movement
              ''')
        user_input = input('Enter command: ')
        # Process user input
        match user_input:
            case 'HOME':
                time.sleep(1) # Wait for serial connection to establish
                msg = String()
                msg.data = 'GO_HOME'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(0.5)
            case 'OPEN':
                time.sleep(1) # Wait for serial connection to establish
                msg = String()
                msg.data = 'GO_OPEN'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(0.5)
            case 'DOCK':
                # Start recording to rosbag "docking" + date and time in YYYYMMDDHHMMSS format
                #filename = ' docking_' + time.strftime('%Y%m%d%H%M%S')
                #print(f'Recording to file: {filename}')
                #command = 'ros2 bag record --output ' + filename + ' /abp12_deltapitch /abp12_deltaroll /abp12_deltax /abp12_deltay /abp12_deltayaw /abp12_deltaz /abp1_pose /abp2_pose /abp1/gra_motor_current_iq /abp1/gra_motor_pos /abp1/gra_motor_vel'
                #self.bagprocess = subprocess.Popen([command], stdin=subprocess.PIPE, shell=True, cwd="/home/labpi/docklab2_ws/bag_files", executable='/bin/bash')

                # Run docking procedure
                self.abp1_GRASP_pub_.publish(String(data='GO_CAPTURE'))
                # Delay whilst docking procedure runs
                time.sleep(0.5)
                #self.bagprocess.send_signal(subprocess.signal.SIGINT)
                #self.bagprocess.wait()
            case 'UNDOCK':
                # Start recording to rosbag "docking" + date and time in YYYYMMDDHHMMSS format
                #filename = ' undocking_' + time.strftime('%Y%m%d%H%M%S')
                #print(f'Recording to file: {filename}')
                #command = 'ros2 bag record --output ' + filename + ' /abp12_deltapitch /abp12_deltaroll /abp12_deltax /abp12_deltay /abp12_deltayaw /abp12_deltaz /abp1_pose /abp2_pose /abp1/gra_motor_current_iq /abp1/gra_motor_pos /abp1/gra_motor_vel'
                #self.bagprocess = subprocess.Popen([command], stdin=subprocess.PIPE, shell=True, cwd="/home/labpi/docklab2_ws/bag_files", executable='/bin/bash')

                # Run undocking procedure
                self.abp1_GRASP_pub_.publish(String(data='GO_RELEASE'))
                # Delay whilst undocking procedure runs
                time.sleep(0.5)
                # Send Ctrl+C to the subprocess to terminate it gracefully
                #self.bagprocess.send_signal(subprocess.signal.SIGINT)
                #self.bagprocess.wait()

            case 'RECORD':
                # Start recording to rosbag "recording" + date and time in YYYYMMDDHHMMSS format
                filename = 'recording_' + time.strftime('%Y%m%d%H%M%S')
                # Print name of file to be recorded
                print(f'Recording to file: {filename}')
                command = 'ros2 bag record --output ' + filename + ' /abp12_deltapitch /abp12_deltaroll /abp12_deltax /abp12_deltay /abp12_deltayaw /abp12_deltaz /abp1_pose /abp2_pose /abp1/gra_motor_current_iq /abp1/gra_motor_pos /abp1/gra_motor_vel'
                self.bagprocess = subprocess.Popen([command], stdin=subprocess.PIPE, shell=True, cwd="/home/labpi/docklab2_ws/bag_files", executable='/bin/bash')
                print(f'Recording started: {filename}')
                # Delay for recording
                time.sleep(20)
                # Stop recording to rosbag
                print('Stopping recording...')
                # Send Ctrl+C to the subprocess to terminate it gracefully
                self.bagprocess.send_signal(subprocess.signal.SIGINT)
                self.bagprocess.wait()

            case 'STOP' | 'S' | 's' | 'stop' :
                # Stop motor movement.
                self.abp1_GRASP_pub_.publish(String(data='STOP'))
                time.sleep(0.5)

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