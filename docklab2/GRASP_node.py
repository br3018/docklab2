#!/usr/bin/env python

# Importing ROS2 Python Client Libraries 
import rclpy
from rclpy.node import Node

# Import service type 
from std_srvs.srv import Trigger

#from docklab2.srv import MoveMotor

# Import message type 
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int32

# Import Python libraries 
import SoloPy as solo
# if solopy is not found in ROS2, need to run this before 
# export PYTHONPATH=/home/labpi/py_env/lib/python3.12/site-packages:$PYTHONPATH
import serial
import time
#import csv
#import os
from datetime import datetime

print(solo.__file__)

class GRASPNode(Node):
    # Constructor
    def __init__(self):
        # Set node name 
        super().__init__('GRASP_node')
        # Set up motor controller instances


        # -----------------------Improvement to be done once we figure it out out tochange the adress in each board:  -----------------------
        # Understand which board is which:
        
        #Solo_1 = solo.SoloMotorControllerUart(port="/dev/ttyACM0",baudrate=solo.UartBaudRate.RATE_937500) #This port connection worked.
        #Solo_2 = solo.SoloMotorControllerUart(port="/dev/ttyACM0",baudrate=solo.UartBaudRate.RATE_937500) #This port connection worked.
        
        #try:    
        #    solo1_adress, error = Solo_1.get_device_address()[0] 
        #except:
        #    self.get_logger().warning('Tried to communicate with port /dev/ttyACM0 but no board is connected there')
            
        
        #solo1_adress, error = Solo_1.get_device_address()[0]
        #solo1_adress, error = Solo_2.get_device_address()[0]
        
        #if solo1_adress1 == 3 and solo2_adress == 7:
        #    self.avc_Solo = Solo_1
        #    self.grapple_Solo = Solo_2
        #elif solo1_adress1 == 7 and solo2_adress == 3:
        #    self.avc_Solo = Solo_2
        #    self.grapple_Solo = Solo_1
        #else:
        #    print('ERROR, not recognized board address, 1 board should have 1')
        # ---------------------------------------------------------------------------------------------------------------------------------
        
        print("\n\n\n\n\ connecting with GRAPPLE \n\n\n\n\n\n\n\n\n")  
        self.grapple_Solo         = self.grapple_motor_init()
        #self.avc_Solo     = self.avc_motor_init()

        
        
        #set up function that runs constatntly monitoring the state machine
        self.timer = self.create_timer(0.1, self.GRASP_state_machine)
        
        
        # Add serial close service
        #self.srv = self.create_service(Trigger, 'grapple_init', self.grapple_init_callback)
        #self.srv = self.create_service(Trigger, 'grapple_init', self.avc_init_callback)
        
        
        # Add GRASP state subscribers
        self.subscription = self.create_subscription(String, '/GRASP_flags',               self.GRASP_external_flags, 10) #QoS arbitrarily set at 10
        self.subscription = self.create_subscription(Float64,'/grapple_motor/position_cmd',self.grapple_motor_position_control,10)
        self.subscription = self.create_subscription(Float64,'/grapple_motor/velocity_cmd',self.grapple_motor_speed_control,10)
        self.subscription = self.create_subscription(Float64,'/grapple_motor/torque_cmd',  self.grapple_motor_torque_control,10)
        self.subscription = self.create_subscription(Float64,'/avc_motor/position_cmd',self.avc_motor_position_control,10)
        self.subscription = self.create_subscription(Float64,'/avc_motor/velocity_cmd',self.avc_motor_speed_control,10)
        self.subscription = self.create_subscription(Float64,'/avc_motor/torque_cmd',  self.avc_motor_torque_control,10)
        self.subscription # prevent unused variable error
        
        self.pub_gra_motor_feedback = self.create_publisher(String,  'gra_motor_feedback', 10)
        self.pub_gra_motor_curr_iq  = self.create_publisher(Float64, 'gra_motor_current_iq', 10)
        self.pub_gra_motor_pos      = self.create_publisher(Int32,   'gra_motor_pos', 10)
        self.pub_gra_motor_vel      = self.create_publisher(Int32,   'gra_motor_vel', 10)
        self.pub_avc_motor_feedback = self.create_publisher(String,  'avc_motor_feedback', 10)
        self.pub_avc_motor_curr_iq  = self.create_publisher(Float64, 'avc_motor_current_iq', 10)
        self.pub_avc_motor_pos      = self.create_publisher(Int32,   'avc_motor_pos', 10)
        self.pub_avc_motor_vel      = self.create_publisher(Int32,   'avc_motor_vel', 10)
        
        self.get_logger().info('GRASP_node initiated with a periodic state machine, and state set to HOME.')
        
        self.grapple_state = 'IDLE'
        self.avc_state     = 'AVC_IDLE'

        # To be added still. A lookup table of all the motor position in quadpulses for HOME/OPEN/etc..
        # LUT with GRAPPLE desired positions in quadpulses
        #self.GRAPPLE_POS_LUT.HOME      = 0
        #self.GRAPPLE_POS_LUT.HARD_DOCK = 549   # From Rhys numbers we had 549.12   but quadpulses in solo is an Int, we can't have decimals
        #self.GRAPPLE_POS_LUT.OPEN      = 41020 # From Rhys numbers we had 41020.32 but quadpulses in solo is an Int, we can't have decimals
        #LETS make sure he numbers are correct
        #TEMPORARY NUMBERS FOR TESTING
        #self.GRAPPLE_POS_LUT.HOME      = 0
        #self.GRAPPLE_POS_LUT.HARD_DOCK = 549   
        #self.GRAPPLE_POS_LUT.OPEN      = 5000 
        #self.GRAPPLE_POS_LUT.RELEASE   = 5000 

    # ================================= GRAPPLE motor initialization ==============================================
    def grapple_motor_init(self):
    
        #Connect with the motor
        
        grapple_Solo = solo.SoloMotorControllerUart(port="/dev/ttyACM0",baudrate=solo.UartBaudRate.RATE_937500) #This port connection worked.
        
        print('Reseting position to zero:')
        grapple_Solo.reset_position_to_zero()
        
        print(f"GRAPPPLE device address: {grapple_Solo.get_device_address()[0]}")
        
        print("--------------- GRAPPLE MOTOR ------------------")
        print("---- Position Reference: "       + str(grapple_Solo.get_position_reference()[0]) )
        print("---- Position Counts feedback: " + str(grapple_Solo.get_position_counts_feedback()[0]) )
        #print("Is the motor enabled: " + str(grapple_Solo.is_drive_enabled() ))
        
        if grapple_Solo.connect():
            self.get_logger().info(f"Connected to GRAPPLE motor")
            
            # Initial Configuration of the device and the Motor
            grapple_Solo.set_output_pwm_frequency_khz(20)              # Desired Switching or PWM Frequency at Output
            grapple_Solo.set_current_limit(3)                          # Current Limit of the Motor
            grapple_Solo.set_motor_poles_counts(4)                     # Motor's Number of Poles
            grapple_Solo.set_command_mode(solo.CommandMode.DIGITAL)    # 
            grapple_Solo.set_motor_type(solo.MotorType.BLDC_PMSM)
            grapple_Solo.set_feedback_control_mode(solo.FeedbackControlMode.HALL_SENSORS)
            grapple_Solo.set_current_controller_kp(1.2)                # current controller Kp
            grapple_Solo.set_current_controller_ki(0.014)              # current controller Ki
            grapple_Solo.set_speed_controller_kp(0.2)                  # Speed controller Kp
            grapple_Solo.set_speed_controller_ki(0.004)                # Speed controller Ki
            grapple_Solo.set_position_controller_kp(20)                # Position controller Kp
            grapple_Solo.set_position_controller_ki(0)                 # Position controller Ki
            grapple_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
            grapple_Solo.set_speed_limit(4750)                         # Desired Speed Limit[RPM].
            grapple_Solo.set_speed_acceleration_value(100)            # Speed acceleration limit[RPM].
            grapple_Solo.set_speed_deceleration_value(100)            # Speed deceleration limit[RPM].
            
            print(f"   RAPPLE Board Temperature: {str(grapple_Solo.get_board_temperature()[0])} degrees")  
            print("    The position controller gains for the GRAPPLE motor are:")
            print("        Kp = " + str(grapple_Solo.get_position_controller_kp()[0] ) )
            print("     OPEN   Ki = " + str(grapple_Solo.get_position_controller_ki()[0] ) )
            print("    The velocity controller gains are:")
            print("        Kp = " + str(grapple_Solo.get_speed_controller_kp()[0] ) )
            print("        Ki = " + str(grapple_Solo.get_speed_controller_ki()[0] ) )
            print("    The current controller gains are:")
            print("        Kp = " + str(grapple_Solo.get_current_controller_kp()[0] ) )
            print("        Ki = " + str(grapple_Solo.get_current_controller_ki()[0] ) )
            print("    Control mode is: " + str(grapple_Solo.get_control_mode()[0] ) )
            print("    Speed Limit is: " + str(grapple_Solo.get_speed_limit()[0] ) )
            print("    Current Limit is: " + str(grapple_Solo.get_current_limit()[0] ) )
            print("    Speed acceleration limit is: " + str(grapple_Solo.get_speed_acceleration_value()[0] ) )
            
            self.grapple_state = 'IDLE'
            self.gra_motor_pos = grapple_Solo.get_position_counts_feedback()[0]
            self.gra_motor_control_mode = "POSITION"
            
        else:
            self.get_logger().warning('Failed to establish connection with GRAPPLE Solo Motor Controller')
        return grapple_Solo


    # ================================= AVC motor initialization ==============================================
    def avc_motor_init(self):
    
        #Connect with the motor
        
        avc_Solo = solo.SoloMotorControllerUart(port="/dev/ttyACM1",baudrate=solo.UartBaudRate.RATE_937500) #This port connection worked.
        
        print('Reseting position to zero:')
        avc_Solo.reset_position_to_zero()
        
        print(f"AVC device address: {avc_Solo.get_device_address()[0]}")
        
        print("--------------- AVC MOTOR ------------------")
        print("---- Position Reference: "       + str(avc_Solo.get_position_reference()[0]) )
        print("---- Position Counts feedback: " + str(avc_Solo.get_position_counts_feedback()[0]) )
        
        
        if avc_Solo.connect():
            self.get_logger().info(f"Connected to AVC motor")
            
            # Initial Configuration of the device and the Motor
            avc_Solo.set_output_pwm_frequency_khz(20)              # Desired Switching or PWM Frequency at Output
            avc_Solo.set_current_limit(3)                          # Current Limit of the Motor
            avc_Solo.set_motor_poles_counts(4)                     # Motor's Number of Poles
            avc_Solo.set_command_mode(solo.CommandMode.DIGITAL)    # 
            avc_Solo.set_motor_type(solo.MotorType.BLDC_PMSM)
            avc_Solo.set_feedback_control_mode(solo.FeedbackControlMode.HALL_SENSORS)
            avc_Solo.set_current_controller_kp(3.397499)           # current controller Kp
            avc_Solo.set_current_controller_ki(0.0158462)          # Current controller Ki
            avc_Solo.set_speed_controller_kp(0.0099868)            # Speed controller Kp
            avc_Solo.set_speed_controller_ki(0.0029907)            # Speed controller Ki
            avc_Solo.set_position_controller_kp(2)                 # Position controller Kp
            avc_Solo.set_position_controller_ki(0.0099868)         # Position controller Ki
            avc_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
            avc_Solo.set_speed_limit(4000)                         # Desired Speed Limit[RPM].
            
            print(f"   AVC Board Temperature: {str(avc_Solo.get_board_temperature()[0])} degrees")  
            print("    The position controller gains for the AVC motor are:")
            print("        Kp = " + str(avc_Solo.get_position_controller_kp()[0] ) )
            print("     OPEN   Ki = " + str(avc_Solo.get_position_controller_ki()[0] ) )
            print("    The velocity controller gains are:")
            print("        Kp = " + str(avc_Solo.get_speed_controller_kp()[0] ) )
            print("        Ki = " + str(avc_Solo.get_speed_controller_ki()[0] ) )
            print("    The current controller gains are:")
            print("        Kp = " + str(avc_Solo.get_current_controller_kp()[0] ) )
            print("        Ki = " + str(avc_Solo.get_current_controller_ki()[0] ) )
            print("    Control mode is: " + str(avc_Solo.get_control_mode()[0] ) )
            print("    Speed Limit is: " + str(avc_Solo.get_speed_limit()[0] ) )
            print("    Current Limit is: " + str(avc_Solo.get_current_limit()[0] ) )
            
            self.avc_state = 'AVC_IDLE'
            self.avc_pos = avc_Solo.get_position_counts_feedback()[0]
            self.gra_motor_control_mode = "POSITION"
        else:
            self.get_logger().warning('Failed to establish connection with AVC Solo Motor Controller')
        return avc_Solo
        
    
    #======================== functions to interact directly with GRAPPLE motor for pos, speed and torque control ==============================
    def grapple_motor_position_control(self, msg):
        # This a function to request a specific position to the motor. !!!!! This is a debugging function. Won't be needed in the final code version.!!!!! 
        
        position_ref = msg.data
        self.grapple_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
        self.gra_motor_control_mode = "POSITION"
        self.get_logger().info(f"POSITION MODE: Commanding the motor to rotate to position: {position_ref}")
        self.grapple_Solo.set_position_reference(position_ref)
        
        
    def grapple_motor_speed_control(self, msg):
        # This a function to request a specific speed to the motor. !!!!! This is a debugging function. Won't be needed in the final code version.!!!!! 
        
        #C.W (Clockwise), is going outwards, i.r,is negative quadpulses
        velocity_input = msg.data
        
        #set_motor direction
        if velocity_input >= 0:
            print('The velocity is positive, and we want to go go COUNTER CLOCKWISE')
            # Velocity is positive, so we want to go COUNTERCLOCKWISE
            direction = solo.Direction.COUNTERCLOCKWISE
            
        else:
            print('The velocity is negative, and we want to go CLOCKWISE')
            # Velocity is negative, so we want to go CLOCKWISE
            direction = solo.Direction.CLOCKWISE
            
        self.grapple_Solo.set_control_mode(solo.ControlMode.SPEED_MODE)
        self.grapple_Solo.set_motor_direction(direction)
        
        velocity_ref = abs(velocity_input)
        
        #Set control mode to SPEED
        self.gra_motor_control_mode = "SPEED"
        # Command speed reference
        self.grapple_Solo.set_speed_reference(velocity_ref)
        self.get_logger().info(f"VELOCITY MODE: Commanding the motor to rotate {direction} at velocity: {velocity_ref}")
        
        
    def grapple_motor_torque_control(self, msg):
        # This a function to request a specific torque to the motor. !!!!! This is a debugging function. Won't be needed in the final code version.!!!!! 
        
        torque_input = msg.data
        
        #set_motor direction
        if torque_input >= 0:
            print('The torque is positive, so we want to go go COUNTER CLOCKWISE')
            # Velocity is positive, so we want to go COUNTERCLOCKWISE
            self.grapple_Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
        else:
            print('The torque is negative, so we want to go CLOCKWISE')
            # Velocity is negative, so we want to go CLOCKWISE
            self.grapple_Solo.set_motor_direction(solo.Direction.CLOCKWISE)
        
        torque_ref = abs(torque_input)
        
        self.grapple_Solo.set_control_mode(solo.ControlMode.TORQUE_MODE)
        self.gra_motor_control_mode = "TORQUE"
        self.get_logger().info(f"TORQUE MODE: Commanding the motor to rotate with torque: {torque_ref}")
        self.grapple_Solo.set_torque_reference_iq(torque_ref)
    
    
    #======================== functions to interact directy with AVC motor for pos, speed and torque control ==============================
    def avc_motor_position_control(self, msg):
        # This a function to request a specific position to the motor. !!!!! This is a debugging function. Won't be needed in the final code version.!!!!! 
        
        position_ref = msg.data
        self.avc_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
        self.avc_motor_control_mode = "POSITION"
        self.get_logger().info(f"POSITION MODE: Commanding the motor to rotate to position: {position_ref}")
        self.avc_Solo.set_position_reference(position_ref)
        
        
    def avc_motor_speed_control(self, msg):
        # This a function to request a specific speed to the motor. !!!!! This is a debugging function. Won't be needed in the final code version.!!!!! 
        
        #C.W (Clockwise), is going outwards, i.r,is negative quadpulses
        velocity_input = msg.data
        
        #set_motor direction
        if velocity_input >= 0:
            print('The velocity is positive, and we want to go go COUNTER CLOCKWISE')
            # Velocity is positive, so we want to go COUNTERCLOCKWISE
            direction = solo.Direction.COUNTERCLOCKWISE
            
        else:
            print('The velocity is negative, and we want to go CLOCKWISE')
            # Velocity is negative, so we want to go CLOCKWISE
            direction = solo.Direction.CLOCKWISE
            
        self.avc_Solo.set_control_mode(solo.ControlMode.SPEED_MODE)
        self.avc_Solo.set_motor_direction(direction)
        
        velocity_ref = abs(velocity_input)
        
        #Set control mode to SPEED
        self.avc_motor_control_mode = "SPEED"
        # Command speed reference
        self.avc_Solo.set_speed_reference(velocity_ref)
        self.get_logger().info(f"VELOCITY MODE: Commanding the motor to rotate {direction} at velocity: {velocity_ref}")
        
        
    def avc_motor_torque_control(self, msg):
        # This a function to request a specific torque to the motor. !!!!! This is a debugging function. Won't be needed in the final code version.!!!!! 
        
        torque_input = msg.data
        
        #set_motor direction
        if torque_input >= 0:
            print('The torque is positive, so we want to go go COUNTER CLOCKWISE')
            # Velocity is positive, so we want to go COUNTERCLOCKWISE
            self.avc_Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
        else:
            print('The torque is negative, so we want to go CLOCKWISE')
            # Velocity is negative, so we want to go CLOCKWISE
            self.avc_Solo.set_motor_direction(solo.Direction.CLOCKWISE)
        
        torque_ref = abs(torque_input)
        
        self.avc_Solo.set_control_mode(solo.ControlMode.TORQUE_MODE)
        self.avc_motor_control_mode = "TORQUE"
        self.get_logger().info(f"TORQUE MODE: Commanding the motor to rotate with torque: {torque_ref}")
        self.avc_Solo.set_torque_reference_iq(torque_ref)
        
        

    def serial_disconnect_callback(self, request, response):
        # Tiago's comment: Still haven't looked what this function is for and if we need it, but was here before so won't remove it for now.
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
        
    # ======================= Function to publish data into topics =========================================
    def publish_data(self):
        # This function publish the data we want to record for external analysis
        
        # ----------------- GRAPPLE_DATA ----------------------------------
        msg = String()
        msg.data = f"State: {self.grapple_state}, Pos ref: {self.gra_motor_pos_ref}, Pos counts: {self.gra_motor_pos},Speed: {self.gra_motor_speed},Current Iq: {self.gra_motor_current}"
        self.pub_gra_motor_feedback.publish(msg)

        #Position Feedback
        motor_pos_msg = Int32()
        motor_pos_msg.data = self.gra_motor_pos
        self.pub_gra_motor_pos.publish(motor_pos_msg)
        
        #Velocity Feedback
        motor_vel_msg = Int32()
        motor_vel_msg.data = self.gra_motor_speed
        self.pub_gra_motor_pos.publish(motor_vel_msg)
        
        #Current Phase Iq Feedback
        current_iq_msg = Float64()
        current_iq_msg.data = self.gra_motor_current
        self.pub_gra_motor_curr_iq.publish(current_iq_msg)
        
        '''
        # ----------------- AVC_DATA ----------------------------------
        avc_msg = String()
        avc_msg.data = f"State: {self.avc_state}, Pos ref: {self.avc_motor_pos_ref}, Pos counts: {self.avc_motor_pos},Speed: {self.avc_motor_speed},Current Iq: {self.avc_motor_current}"
        self.pub_avc_motor_feedback.publish(avc_msg)

        #Position Feedback
        avc_motor_pos_msg = Int32()
        avc_motor_pos_msg.data = self.avc_motor_pos
        self.pub_avc_motor_pos.publish(avc_motor_pos_msg)
        
        #Velocity Feedback
        avc_motor_vel_msg = Int32()
        avc_motor_vel_msg.data = self.avc_motor_speed
        self.pub_avc_motor_pos.publish(avc_motor_vel_msg)
        
        #Current Phase Iq Feedback
        avc_current_iq_msg = Float64()
        avc_current_iq_msg.data = self.avc_motor_current
        self.pub_avc_motor_curr_iq.publish(avc_current_iq_msg)
        '''
        
    # ================================= External flags management code ==============================================    
    def GRASP_external_flags(self,msg):
        # This functions runs when we receive an external flag through the topic /GRASP_flags
        # This means that this code does not run in loop. It's only triggered with the flags, like an interruption. 
        # We first will match the received flag and then perform the associate
    
        flag = msg.data   
        
        match flag:
        
            case 'GO_OPEN':
                
                if self.grapple_state != 'HOME':
                    # There must be an error. Previous state has to be HOME, or motor position is not zero!
                    # We should abort
                    self.get_logger().warning("Failed to change mode to 'OPEN' because previous state isn't HOME. ")
                    return
        
        
                self.grapple_Solo.set_speed_acceleration_value(100)            # Speed acceleration limit[RPM].
                self.grapple_Solo.set_speed_deceleration_value(100)            # Speed deceleration limit[RPM].
                
                self.grapple_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                self.gra_motor_control_mode = "POSITION"
                
                # Set position reference to -5000.
                #self.target_pos = -5000
                self.target_pos = -41000
                
                self.get_logger().info(f"Commanding now grapple_motor to rotate to position {self.target_pos}")
                self.grapple_Solo.set_position_reference(self.target_pos)
                self.grapple_state = 'OPEN'
                
            case 'GO_CAPTURE':
            
                if self.grapple_state != 'OPEN':
                    # There must be an error. Previous state has to be HOME.
                    # We should abort
                    self.get_logger().warning("Failed to change mode to 'CAPTURE' because previous state isn't OPEN. ")
                    return
                
                self.grapple_Solo.set_speed_acceleration_value(100)            # Speed acceleration limit[RPM].
                self.grapple_Solo.set_speed_deceleration_value(100)            # Speed deceleration limit[RPM].
                
                # change to velocity mode
                self.grapple_Solo.set_control_mode(solo.ControlMode.SPEED_MODE) 
                self.gra_motor_control_mode = "SPEED"
                
                # Set motor spin direction, we want to close GRASP, which is moving motor in COUNTER CLOCK WISE
                self.grapple_Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
                
                speed_ref = 4750
                #speed_ref = 3000
                
                self.grapple_Solo.set_speed_reference(speed_ref)
                self.grapple_state = 'CAPTURING'
                
                ## Set position reference
                #capture_pos_delta = 500
                
                #initial_pos, error = self.grapple_Solo.get_position_counts_feedback();
                #self.target_pos  = initial_pos + capture_pos_delta
                
                #self.get_logger().info(f"Position counts feedback: {initial_pos}.")
                #self.get_logger().info(f"Commanding now grapple_motor to rotate to position {self.target_pos}")
                ##self.grapple_Solo.set_position_reference(self.target_pos)
                
            
            case 'GO_HOME':
                # change to velocity mode
                self.grapple_Solo.set_control_mode(solo.ControlMode.TORQUE_MODE) 
                self.gra_motor_control_mode = "TORQUE"
                # Set motor spin direction, we want to close GRASP, which is moving motor in COUNTER CLOCK WISE
                self.grapple_Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
                
                torque_ref = 2
                #torque_ref = 0.2
                
                self.grapple_Solo.set_torque_reference_iq(torque_ref)
                self.grapple_state = "HOMING"
                
                
            case 'GO_RELEASE':
            
                if self.grapple_state != 'HARD_DOCK':
                    # There must be an error. Previous state has to be HOME.
                    # We should abort
                    self.get_logger().warning("Failed to change mode to 'OPEN' because previous state isn't HARD_DOCK. ")
                    return
        
        
                self.grapple_Solo.set_speed_limit(4750)                         # Desired Speed Limit[RPM].
                self.grapple_Solo.set_speed_acceleration_value(10)            # Speed acceleration limit[RPM].
                self.grapple_Solo.set_speed_deceleration_value(10)            # Speed deceleration limit[RPM].
                
                print(f"Reading speed_deceleration value:  {self.grapple_Solo.get_speed_deceleration_value()[0]}")            # Speed deceleration limit[RPM].
                
                #self.target_pos  = -20000
                self.target_pos  = -41000   # This would be the open position
                self.gra_motor_control_mode = "POSITION"
                
                self.grapple_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                self.grapple_Solo.set_position_reference(self.target_pos)
                
                self.get_logger().info(f"Commanding now grapple_motor to rotate to position {self.target_pos}")
                
                self.grapple_state = 'RELEASE'
                
                
            case 'STOP':
            
                control_mode, error = self.grapple_Solo.get_control_mode()
                
                if control_mode == solo.ControlMode.POSITION_MODE or control_mode == solo.ControlMode.SPEED_MODE:
                    self.grapple_Solo.set_control_mode(solo.ControlMode.SPEED_MODE)
                    self.gra_motor_control_mode = "SPEED"
                    self.grapple_Solo.set_speed_reference(0)
                    print('Asked the motor top STOP by setting SPEED to zero.')
                    self.grapple_state = 'IDLE'
                elif control_mode == solo.ControlMode.TORQUE_MODE:
                    self.grapple_Solo.set_control_mode(solo.ControlMode.TORQUE_MODE)
                    self.gra_motor_control_mode = "TORQUE"
                    self.gcurrent_thresholdet_logger().info(f"TORQUE MODE: Commanding the motor to stop by setting TORQUE to zero.")
                    self.grapple_Solo.set_torque_reference_iq(0)
                else:
                    # Do the same for velocity stop
                    self.grapple_Solo.set_control_mode(solo.ControlMode.SPEED_MODE)
                    self.gra_motor_control_mode = "SPEED"
                    self.grapple_Solo.set_speed_reference(0)
                    print('Asked the motor top STOP by setting SPEED to zero.')
                
                self.grapple_state = 'IDLE'
                
            case 'RESET':
            
                print('Resetting position to zero:')
                self.get_logger().info(f"Resetting position to zero")
                self.grapple_Solo.reset_position_to_zero()
                self.grapple_Solo.set_position_reference(0)
                self.grapple_state = 'IDLE'
                
            case 'GO_AVC_HOMING':
                print('comanding AVC to perform homing.')
                
                # change to torqe mode
                self.avc_Solo.set_control_mode(solo.ControlMode.SPEED_MODE) 
                self.avc_motor_control_mode = "SPEED"
                # Set motor spin direction, we want to close GRASP, which is moving motor in COUNTER CLOCK WISE
                self.grapple_Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
                
                speed_ref = 1000
                
                self.avc_Solo.set_speed_reference(speed_ref)
                self.avc_state = "AVC_HOMING"
                
            case 'GO_POS1':
                
                if self.avc_state != 'AVC_HOME':
                    # There must be an error. Previous state has to be HOME, or motor position is not zero!
                    # We should abort
                    self.get_logger().warning("Failed to change mode to 'OPEN' because previous state isn't AVC_HOME. ")
                    return
        
                self.avc_Solo.set_speed_acceleration_value(100)            # Speed acceleration limit[RPM].
                self.avc_Solo.set_speed_deceleration_value(100)            # Speed deceleration limit[RPM].
                
                self.avc_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                self.avc_motor_control_mode = "POSITION"
                
                # Set position reference to -5000.
                self.target_pos = -1000
                
                
                self.get_logger().info(f"Commanding now avc_motor to rotate to position {self.target_pos}")
                self.avc_Solo.set_position_reference(self.target_pos)
                self.avc_Solo = 'POS1'
                
            case 'GO_POS2':
                
                if self.avc_state != 'AVC_POS1':
                    # There must be an error. Previous state has to be HOME, or motor position is not zero!
                    # We should abort
                    self.get_logger().warning("Failed to change mode to 'OPEN' because previous state isn't AVC_HOME. ")
                    return
        
                self.avc_Solo.set_speed_acceleration_value(100)            # Speed acceleration limit[RPM].
                self.avc_Solo.set_speed_deceleration_value(100)            # Speed deceleration limit[RPM].
                
                self.avc_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                self.avc_motor_control_mode = "POSITION"
                
                # Set position reference to -5000.
                self.target_pos = -1500
                
                self.get_logger().info(f"Commanding now avc_motor to rotate to position {self.target_pos}")
                self.avc_Solo.set_position_reference(self.target_pos)
                self.avc_Solo = 'POS2'
                
            
            case 'GO_POS1.5':
                if self.avc_state != 'AVC_POS2':
                    # There must be an error. Previous state has to be HOME, or motor position is not zero!
                    # We should abort
                    self.get_logger().warning("Failed to change mode to 'OPEN' because previous state isn't AVC_HOME. ")
                    return
        
                self.avc_Solo.set_speed_acceleration_value(100)            # Speed acceleration limit[RPM].
                self.avc_Solo.set_speed_deceleration_value(100)            # Speed deceleration limit[RPM].
                
                self.avc_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                self.avc_motor_control_mode = "POSITION"
                
                # Set position reference to -5000.
                self.target_pos = -1250
                
                self.get_logger().info(f"Commanding now avc_motor to rotate to position {self.target_pos}")
                self.avc_Solo.set_position_reference(self.target_pos)
                self.avc_Solo = 'POS1.5'
                
    # ============================= State machine code, runs different parts of script based on current state ===============================  
    def GRASP_state_machine(self):
        # This function manages the state machine of GRASP CONOPS. This function is run in a loop,so that we are always are of which state we
        
        #time_0 = time.time()
        # read data from the motor
             
        self.gra_motor_pos_ref, error = self.grapple_Solo.get_position_reference()
        self.gra_motor_pos, error     = self.grapple_Solo.get_position_counts_feedback()
        self.gra_motor_speed, error   = self.grapple_Solo.get_speed_feedback()
        self.gra_motor_current, error = self.grapple_Solo.get_quadrature_current_iq_feedback()

        # --------------  Checking GRAPPLE states and calling relevant code. ---------------------------  
        match self.grapple_state:
            
            case "IDLE":
                # This grapple_state is the first mode we go to when GRASP is booted for the first time
                state = "0"

            case "HOMING":
                state = "1"
                # We are moving in this state, we moving in the positive direction (closing arms)
                current_threshold = 2
                print('In MODE HOMING')
                # Measure 
                
                if abs(self.gra_motor_current) > current_threshold:
                    
                    #We have reached home, so let's stop the motor from spinning
                    self.grapple_Solo.set_control_mode(solo.ControlMode.TORQUE_MODE)
                    self.gra_motor_control_mode = "TORQUE"
                    self.grapple_Solo.set_torque_reference_iq(0)
                    print('THRESHOLD HAS BEEN REACHED!!')
                    print('Asked the motor top STOP by setting TORQUE to zero.')
                    self.get_logger().info(f"GRAPPLE has reached a current limit! commanded the motor to stop by setting torque to zero.")
                    time.sleep(0.5)                   #Pausing/waiting some seconds before we reset the position back to zero, because resetting the position is a hard_stop.
                    
                    
                    #Reading again the feedback from motor because we used the pausing feature: time.sleep()
                    self.gra_motor_pos_ref, error = self.grapple_Solo.get_position_reference()
                    self.gra_motor_pos, error     = self.grapple_Solo.get_position_counts_feedback()
                    self.gra_motor_speed, error   = self.grapple_Solo.get_speed_feedback()
                    self.gra_motor_current, error = self.grapple_Solo.get_quadrature_current_iq_feedback()
                    print('reset home position')
                    #Resetting position back to zero.
                    #self.grapple_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                    #self.motor_control_mode = "POSITION"
                    self.grapple_Solo.reset_position_to_zero()
                    self.grapple_Solo.set_position_reference(0)
                    
                    self.grapple_state = 'HOME'
                    
                    
            case "HOME":
                # We do nothing in this state. We are just waiting for the "GO_OPEN" flag to come in
                state = "2"
                
            case "OPEN":
                state = "3"
                # We do nothing in this state. Just waiting for external flag to command 
                
            case "CAPTURING":
                
                current_threshold = 2
                print('In MODE CAPTURING')
                # Measure 
                
                if abs(self.gra_motor_current) > current_threshold:
                    
                    #We have reached home, so let's stop the motor from spinning
                    self.grapple_Solo.set_control_mode(solo.ControlMode.SPEED_MODE)
                    self.gra_motor_control_mode = "SPEED"
                    self.grapple_Solo.set_speed_reference(0)
                    print('THRESHOLD HAS BEEN REACHED!!')
                    print('Asked the motor top STOP by setting SPEED to zero.')
                    self.get_logger().info(f"GRAPPLE has reached a current limit! commanded the motor to stop by setting vel to zero.")
                    
                    
                    time.sleep(2)                   #Pausing/waiting some seconds before we reset the position back to zero, because resetting the position is a hard_stop.
                    
                    self.gra_motor_pos, error     = self.grapple_Solo.get_position_counts_feedback()
                    # Let's set the position reference as the current position.
                    self.grapple_Solo.set_position_reference(self.gra_motor_pos)
                    
                    #Reading again the feedback from motor because we used the pausing feature: time.sleep()
                    self.gra_motor_pos_ref, error = self.grapple_Solo.get_position_reference()
                    self.gra_motor_speed, error   = self.grapple_Solo.get_speed_feedback()
                    self.gra_motor_current, error = self.grapple_Solo.get_quadrature_current_iq_feedback()
                    
                    self.grapple_state = 'HARD_DOCK'
                    
                    
            case "HARD_DOCK":
                state = "5"
                # We do nothing in this state. Just waiting for external flag to command to Re-open.

                
                
            case "RELEASE":
                state = "7"
                self.get_logger().info('Updating GRASP state to RELEASE')
                #self.ser.write(state.encode())
                
            case "MORE":
                state = "9"
                self.get_logger().info('Updating GRASP state to MORE')
                #self.ser.write(state.encode())
            case "LESS":
                state = "10"
                self.get_logger().info('Updating GRASP state to LESS')
                #self.ser.write(state.encode())

            case "FORWARD":
                state = "12"
                self.get_logger().info('Updating GRASP state to AVC_FORWARD')
                #self.ser.write(state.encode())
            case "BACK":
                state = "11"
                self.get_logger().info('Updating GRASP state to AVC_BACK')
                #self.ser.write(state.encode())
            case _: # Catch invalid command 
                self.get_logger().warning('Unknown GRASP state received')
        '''   
        # --------------  Checking AVC states and calling relevant code. ---------------------------  
        if self.grapple_state == "HARD_DOCK":
        # If we are in HARD_DOCK, we can run the AVC code
            self.avc_motor_pos_ref, error = self.avc_Solo.get_position_reference()
            self.avc_motor_pos, error     = self.avc_Solo.get_position_counts_feedback()
            self.avc_motor_speed, error   = self.avc_Solo.get_speed_feedback()
            self.avc_motor_current, error = self.avc_Solo.get_quadrature_current_iq_feedback()
        
            match self.avc_state:
                
                case "AVC_IDLE":
                    state = "1"
                    # We do nothing on this state. just wainting for external flag "GO_AVC_HOME" to be sent.
                case "AVC_HOMING":
                    state = "1"
                    # We are moving in this state, we moving in the positive direction (closing arms)
                    avc_current_threshold = 0.1
                    print('In MODE AVC_HOMING')
                    
                    # Measure 
                    
                    if abs(self.avc_motor_current) > avc_current_threshold:
                        
                        #We have reached home, so let's stop the motor from spinning
                        self.avc_solo.set_control_mode(solo.ControlMode.TORQUE_MODE)
                        self.avc_motor_control_mode = "TORQUE"
                        self.avc_solo.set_torque_reference_iq(0)
                        #self.avc_motor_pos, error     = self.avc_solo.get_position_counts_feedback()
                        #self.avc_solo.set_position_reference(self.avc_motor_pos)
                        
                        
                        print('THRESHOLD HAS BEEN REACHED!!')
                        print('Asked the AVC motor top STOP by setting torque to zero.')
                        self.get_logger().info(f"GRAPPLE has reached a current limit! commanded the motor to stop by setting torque to zero.")
                        time.sleep(0.5)                   #Pausing/waiting some seconds before we reset the position back to zero, because resetting the position is a hard_stop.
                        
                        
                        #Reading again the feedback from motor because we used the pausing feature: time.sleep()
                        self.avc_motor_pos_ref, error = self.avc_solo.get_position_reference()
                        self.avc_motor_pos, error     = self.avc_solo.get_position_counts_feedback()
                        self.avc_motor_speed, error   = self.avc_solo.get_speed_feedback()
                        self.avc_motor_current, error = self.avc_solo.get_quadrature_current_iq_feedback()
                        print('reset AVC home position')
                        #Resetting position back to zero.
                        self.avc_solo.reset_position_to_zero()
                        self.avc_solo.set_position_reference(0)
                        
                case "AVC_HOME":
                    #We do nothing on this state. Just waitnig to receivecommand to go pos1
                    state = "11"
                    
                case "AVC_POS1":
                    #We do nothing on this state. Just waitnig to receivecommand to go pos1
                    state = "12"
                case "AVC_POS1.5":
                    #We do nothing on this state. Just waitnig to receivecommand to go pos1
                    state = "13"
                case "AVC_POS2":
                    #We do nothing on this state. Just waitnig to receivecommand to go pos1
                    state = "14"
        else:
        # In the else section, we are not in  HARD_DOCK mode
            self.avc_motor_pos_ref = 0
            self.avc_motor_pos = 0
            self.avc_motor_speed = 0
            self.avc_motor_current = 0.0
        '''
        
        #time_1 = time.time()
        # Publish data
        self.publish_data()
        #time_2 = time.time()
        
        #print(time_1 - time_0)
        #print(time_2 - time_1)
        #print(time_2 - time_0)
        

def main():
    rclpy.init()
    GRASP_node = GRASPNode()
    rclpy.spin(GRASP_node)
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
