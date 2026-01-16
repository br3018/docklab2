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
        self.abp1_GRASP_pub_ = self.create_publisher(String, 'abp1/GRASP_flags', 10) #QoS arbitrarily set at 10
        self.abp1_GRASP_pub_ # prevent unused variable error
        # Setting up client for abp1 namespace services
        self.abp1_airb_cli_ = self.create_client(SetBool, 'abp1/set_Ch1')
        self.abp1_airb_req = SetBool.Request()
        self.abp1_flow_cli_ = self.create_client(SetBool, 'abp1/set_Ch2')
        self.abp1_flow_req = SetBool.Request()
        self.abp1_vent_cli_ = self.create_client(SetBool, 'abp1/set_Ch3')
        self.abp1_vent_req = SetBool.Request() 

        # Setting up ABP2 interfaces
        # Setting up client for abp2 namespace services
        self.abp2_airb_cli_ = self.create_client(SetBool, 'abp2/set_Ch1')
        self.abp2_airb_req = SetBool.Request()

        # Setting up timer for getting user input 
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Get user input
        print('''Command list: 
              "ON":           Switch on air bearings
              "OFF":          Switch off air bearings
              "HOME":         Home GRAPPLE and AVC
              "FREE_FLIGHT":  Set GRAPPLE to free flight mode
              "READY":        Set GRAPPLE to ready mode
              "TRIGGER":      Trigger docking sequence
              "DOCK":         Run docking procedure, begin recording to a rosbag
              "CLEARANCE":    Move GRAPPLE to clearance position
              "AVC_A_POS1":   Send AVC A to POS1 for leak check
              "AVC_A_POS1p5": Send AVC A to POS1.5 for interstitial venting
              "AVC_A_POS2":   Send AVC A to POS2 for fluid transfer
              "AVC_A_RETRACT": Retract AVC A to home position
              "AVC_B_POS1":   Send AVC B to POS1 for leak check
              "AVC_B_POS1p5": Send AVC B to POS1.5 for interstitial venting
              "AVC_B_POS2":   Send AVC B to POS2 for fluid transfer
              "AVC_B_RETRACT": Retract AVC B to home position
              "FLOW_ON":      Open flow valve on ABP1
              "FLOW_OFF":     Close flow valve on ABP1
              "VENT_ON":      Open vent valve on ABP1
              "VENT_OFF":     Close vent valve on ABP1
              "UNDOCK":       Run undocking procedure, begin recording to a rosbag
              "RELEASE":      Release GRAPPLE mechanism
              "RECORD":       Record all topics to a rosbag
              "RESET":        Reset GRASP to uncontrolled mode
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
            case 'FLOW_ON':
                self.abp1_flow_req.data = True
                self.abp1_flow_cli_.call_async(self.abp1_flow_req)
            case 'FLOW_OFF':
                self.abp1_flow_req.data = False
                self.abp1_flow_cli_.call_async(self.abp1_flow_req)
            case 'VENT_ON':
                self.abp1_vent_req.data = True
                self.abp1_vent_cli_.call_async(self.abp1_vent_req)
            case 'VENT_OFF':
                self.abp1_vent_req.data = False
                self.abp1_vent_cli_.call_async(self.abp1_vent_req)
            case 'HOME':
                time.sleep(1) # Wait for serial connection to establish
                msg = String()
                msg.data = 'HOME'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(10)
            case 'FREE_FLIGHT':
                time.sleep(1)
                msg = String()
                msg.data = 'FREE_FLIGHT'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(2)
            case 'READY':
                time.sleep(1)
                msg = String()
                msg.data = 'READY'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(2)
            case 'TRIGGER':
                time.sleep(1)
                msg = String()
                msg.data = 'TRIGGER'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(50)
            case 'CLEARANCE':
                time.sleep(1)
                msg = String()
                msg.data = 'CLEARANCE'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(10)
            case 'DOCK':
                # Start recording to rosbag "docking" + date and time in YYYYMMDDHHMMSS format
                filename = ' docking_' + time.strftime('%Y%m%d%H%M%S')
                print(f'Recording to file: {filename}')
                command = 'ros2 bag record --output ' + filename + ' /abp12_deltapitch /abp12_deltaroll /abp12_deltax /abp12_deltay /abp12_deltayaw /abp12_deltaz /abp1_pose /abp2_pose /abp1/gra_motor_current_iq /abp1/gra_motor_pos /abp1/gra_motor_vel /abp1/gra_motor_feedback'
                self.bagprocess = subprocess.Popen([command], stdin=subprocess.PIPE, shell=True, cwd="/home/labpi/docklab2_ws/bag_files", executable='/bin/bash')

                # Wait a couple of seconds to allow rosbag to start properly
                time.sleep(3)
                
                # Run docking procedure
                # Start air bearings
                self.abp2_airb_req.data = True
                self.abp2_airb_cli_.call_async(self.abp2_airb_req)
                self.abp1_airb_req.data = True
                self.abp1_airb_cli_.call_async(self.abp1_airb_req)
                # Start GRASP
                self.abp1_GRASP_pub_.publish(String(data='TRIGGER'))
                # Delay whilst docking procedure runs
                time.sleep(60)
                # Stop air bearings
                self.abp1_airb_req.data = False
                self.abp1_airb_cli_.call_async(self.abp1_airb_req)
                self.abp2_airb_req.data = False
                self.abp2_airb_cli_.call_async(self.abp2_airb_req)
                # Send Ctrl+C to the subprocess to terminate it gracefully
                self.bagprocess.send_signal(subprocess.signal.SIGINT)
                self.bagprocess.wait()
            case 'AVC_A_POS1':
                time.sleep(1) # Wait for serial communication to establish
                msg = String()
                msg.data = 'AVC_A_POS1'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(10)
            case 'AVC_A_POS2':
                time.sleep(1) # Wait for serial communication to establish
                msg = String()
                msg.data = 'AVC_A_POS2'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(10)
            case 'AVC_A_POS1p5':
                time.sleep(1)
                msg = String()
                msg.data = 'AVC_A_POS1p5'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(10)
            case 'AVC_A_RETRACT':
                time.sleep(1)
                msg = String()
                msg.data = 'AVC_A_RETRACT'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(10)
            case 'AVC_B_POS1':
                time.sleep(1)
                msg = String()
                msg.data = 'AVC_B_POS1'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(10)
            case 'AVC_B_POS2':
                time.sleep(1)
                msg = String()
                msg.data = 'AVC_B_POS2'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(10)
            case 'AVC_B_POS1p5':
                time.sleep(1)
                msg = String()
                msg.data = 'AVC_B_POS1p5'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(10)
            case 'AVC_B_RETRACT':
                time.sleep(1)
                msg = String()
                msg.data = 'AVC_B_RETRACT'
                self.abp1_GRASP_pub_.publish(msg)
                time.sleep(10)
            case 'UNDOCK':
                # Start recording to rosbag "docking" + date and time in YYYYMMDDHHMMSS format
                filename = ' undocking_' + time.strftime('%Y%m%d%H%M%S')
                print(f'Recording to file: {filename}')
                command = 'ros2 bag record --output ' + filename + ' /abp12_deltapitch /abp12_deltaroll /abp12_deltax /abp12_deltay /abp12_deltayaw /abp12_deltaz /abp1_pose /abp2_pose /abp1/gra_motor_current_iq /abp1/gra_motor_pos /abp1/gra_motor_vel /abp1/gra_motor_feedback'
                self.bagprocess = subprocess.Popen([command], stdin=subprocess.PIPE, shell=True, cwd="/home/labpi/docklab2_ws/bag_files", executable='/bin/bash')

                # Run undocking procedure
                # Start air bearings
                self.abp2_airb_req.data = True
                self.abp2_airb_cli_.call_async(self.abp2_airb_req)
                self.abp1_airb_req.data = True
                self.abp1_airb_cli_.call_async(self.abp1_airb_req)
                # Start GRASP release sequence
                self.abp1_GRASP_pub_.publish(String(data='RELEASE'))
                # Delay whilst undocking procedure runs
                time.sleep(60)
                # Stop air bearings
                self.abp1_airb_req.data = False
                self.abp1_airb_cli_.call_async(self.abp1_airb_req)
                self.abp2_airb_req.data = False
                self.abp2_airb_cli_.call_async(self.abp2_airb_req)
                # Send Ctrl+C to the subprocess to terminate it gracefully
                self.bagprocess.send_signal(subprocess.signal.SIGINT)
                self.bagprocess.wait()

            case 'RELEASE':
                time.sleep(1)
                msg = String()
                msg.data = 'RELEASE'
                self.abp1_GRASP_pub_.publish(msg)
                print('RELEASE command sent to GRASP')
                time.sleep(10)

            case 'RECORD':
                # Start recording to rosbag "recording" + date and time in YYYYMMDDHHMMSS format
                filename = 'recording_' + time.strftime('%Y%m%d%H%M%S')
                # Print name of file to be recorded
                print(f'Recording to file: {filename}')
                command = 'ros2 bag record --output ' + filename + ' /abp12_deltapitch /abp12_deltaroll /abp12_deltax /abp12_deltay /abp12_deltayaw /abp12_deltaz /abp1_pose /abp2_pose /abp1/gra_motor_current_iq /abp1/gra_motor_pos /abp1/gra_motor_vel /abp1/gra_motor_feedback'
                self.bagprocess = subprocess.Popen([command], stdin=subprocess.PIPE, shell=True, cwd="/home/labpi/docklab2_ws/bag_files", executable='/bin/bash')
                print(f'Recording started: {filename}')
                # Delay for recording
                time.sleep(20)
                # Stop recording to rosbag
                print('Stopping recording...')
                # Send Ctrl+C to the subprocess to terminate it gracefully
                self.bagprocess.send_signal(subprocess.signal.SIGINT)
                self.bagprocess.wait()

            case 'RESET':
                time.sleep(1)
                msg = String()
                msg.data = 'RESET'
                self.abp1_GRASP_pub_.publish(msg)
                print('RESET command sent to GRASP - now in UNCONTROLLED mode')

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