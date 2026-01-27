# ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== #
# Initial set up

#!/usr/bin/env python

# ROS2 Python Client Libraries 
import rclpy
from rclpy.node import Node

# Service type 
from std_srvs.srv import Trigger

# Message type 
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int32

# Python libraries 
import SoloPy as solo
# If solopy is not found in ROS2, need to run this before: 
# export PYTHONPATH=/home/labpi/py_env/lib/python3.12/site-packages:$PYTHONPATH

import serial
import time
import glob
from datetime import datetime

# Logger
print(solo.__file__)
grapple_logger_level = 1

# Search for available serial devices. 
print('Running a search for available serial devices')
ports_found = 0
while ports_found == 0:
    ports_searched = glob.glob('/dev/ttyACM[0-9]*')
    ports_found = len(ports_searched)
    print('No serial devices found. Awaiting connection.')
    time.sleep(1)
available_ports = []
for port in ports_searched:
    try:
        s = serial.Serial(port)
        s.close()
        available_ports.append(str(port))
        print(f"Found a device active on: {port}")
    except:
        pass

# ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== #
# Class definition for GRASP Node

class GRASPNode(Node):

    def __init__(self):

        super().__init__('GRASP_node')
        
        self.get_logger().info('Initialising GRASP')

        # Initiate GRAPPLE motor driver
        self.grapple_Solo = self.grapple_motor_init()
        
        # Set up a function that constantly monitors the state machine
        self.timer = self.create_timer(0.1, self.GRASP_state_machine)
        
        # Add GRASP state subscribers
        self.subscription = self.create_subscription(String, 'GRASP_flags',                self.GRASP_external_flags, 10) #QoS arbitrarily set at 10
        self.subscription = self.create_subscription(Float64,'grapple_motor/position_cmd', self.grapple_motor_position_control,10)
        self.subscription = self.create_subscription(Float64,'grapple_motor/velocity_cmd', self.grapple_motor_speed_control,10)
        self.subscription = self.create_subscription(Float64,'grapple_motor/torque_cmd',   self.grapple_motor_torque_control,10)
        self.subscription # prevent unused variable error
        
        # Add GRASP publishers
        self.pub_gra_motor_feedback = self.create_publisher(String,  'gra_motor_feedback', 10)
        self.pub_gra_motor_curr_iq  = self.create_publisher(Float64, 'gra_motor_current_iq', 10)
        self.pub_gra_motor_pos      = self.create_publisher(Int32,   'gra_motor_pos', 10)
        self.pub_gra_motor_vel      = self.create_publisher(Int32,   'gra_motor_vel', 10)
        
        self.get_logger().info('GRASP node initiated with a periodic state machine and state set to IDLE.')
        self.grapple_state = 'IDLE'

    # GRAPPLE motor initialization
    def grapple_motor_init(self):

        # Initialise GRAPPLE motor driver
        grapple_connection_successful = 0
        while grapple_connection_successful == 0:
            for grapple_port in available_ports:
                self.get_logger().info(f"Attempting to connect to GRAPPLE motor driver over {grapple_port}")
                grapple_Solo = solo.SoloMotorControllerUart(port=grapple_port, baudrate=solo.UartBaudRate.RATE_937500, address =7, loggerLevel=grapple_logger_level)
                motor_address = grapple_Solo.get_device_address()[0]
                self.get_logger().info(f"{grapple_port} reads back motor address as {motor_address}")
                if motor_address == 7:
                    self.get_logger().info(f"Successfully connected to GRAPPLE motor driver over {grapple_port}")
                    grapple_connection_successful = 1
                    available_ports.remove(grapple_port)
                    break
                else:
                    self.get_logger().warn(f"Could not find GRAPPLE motor driver over {grapple_port}")
                    self.get_logger().warn(f"Disconnecting from {grapple_port}")
                    grapple_Solo.serial_close()
                    grapple_connection_successful = 0

        # Reset initial position to zero
        self.get_logger().info(f"Resetting GRAPPLE position to zero.")
        grapple_Solo.reset_position_to_zero()
        
        if grapple_Solo.connect():
            # Initial Configuration of the device and the Motor
            grapple_Solo.set_output_pwm_frequency_khz(20)              # Desired switching or PWM frequency at output
            grapple_Solo.set_current_limit(3)                          # Current limit of the motor
            grapple_Solo.set_motor_poles_counts(4)                     # Motor's number of poles
            grapple_Solo.set_command_mode(solo.CommandMode.DIGITAL)    # We're using digital command mode instead of analog
            grapple_Solo.set_motor_type(solo.MotorType.BLDC_PMSM)
            grapple_Solo.set_feedback_control_mode(solo.FeedbackControlMode.HALL_SENSORS)
            grapple_Solo.set_current_controller_kp(1.2)                # Current controller Kp
            grapple_Solo.set_current_controller_ki(0.014)              # Current controller Ki
            grapple_Solo.set_speed_controller_kp(0.2)                  # Speed controller Kp
            grapple_Solo.set_speed_controller_ki(0.004)                # Speed controller Ki
            grapple_Solo.set_position_controller_kp(20)                # Position controller Kp
            grapple_Solo.set_position_controller_ki(0)                 # Position controller Ki
            grapple_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
            grapple_Solo.set_speed_limit(4750)                         # Desired Speed Limit[RPM].
            grapple_Solo.set_speed_acceleration_value(100)             # Speed acceleration limit[RPM].
            grapple_Solo.set_speed_deceleration_value(100)             # Speed deceleration limit[RPM].
            
            self.grapple_state = 'IDLE'
            self.get_logger().info("GRAPPLE set to IDLE")
            self.gra_motor_pos = grapple_Solo.get_position_counts_feedback()[0]
            self.gra_motor_control_mode = "POSITION"
        else:
            self.get_logger().error('Failed to establish connection with GRAPPLE Solo Motor Controller')
        return grapple_Solo

    # ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== ===== #
    # Debugging functions to interact directly with GRAPPLE motor for position, speed and torque control
    def grapple_motor_position_control(self, msg):
        position_ref = msg.data
        self.grapple_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
        self.gra_motor_control_mode = "POSITION"
        self.get_logger().info(f"POSITION MODE: Commanding the motor to rotate to position: {position_ref}")
        self.grapple_Solo.set_position_reference(position_ref)

    def grapple_motor_speed_control(self, msg):
        # Clockwise extends GRAPPLE outwards. Corresponds to movement in negative quad pulses.
        velocity_input = msg.data
        
        if velocity_input >= 0:
            print('The velocity is positive, and we want to go COUNTER CLOCKWISE')
            direction = solo.Direction.COUNTERCLOCKWISE
        else:
            print('The velocity is negative, and we want to go CLOCKWISE')
            direction = solo.Direction.CLOCKWISE
            
        self.grapple_Solo.set_control_mode(solo.ControlMode.SPEED_MODE)
        self.grapple_Solo.set_motor_direction(direction)
        velocity_ref = abs(velocity_input)
        self.gra_motor_control_mode = "SPEED"
        self.grapple_Solo.set_speed_reference(velocity_ref)
        self.get_logger().info(f"Commanding the motor to rotate {direction} at velocity: {velocity_ref}")

    def grapple_motor_torque_control(self, msg):
        torque_input = msg.data
        
        if torque_input >= 0:
            print('The torque is positive, so we want to go COUNTER CLOCKWISE')
            # Velocity is positive, so we want to go COUNTERCLOCKWISE
            self.grapple_Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
        else:
            print('The torque is negative, so we want to go CLOCKWISE')
            # Velocity is negative, so we want to go CLOCKWISE
            self.grapple_Solo.set_motor_direction(solo.Direction.CLOCKWISE)
        
        torque_ref = abs(torque_input)
        self.grapple_Solo.set_control_mode(solo.ControlMode.TORQUE_MODE)
        self.gra_motor_control_mode = "TORQUE"
        self.get_logger().info(f"Commanding the motor to rotate with torque: {torque_ref}")
        self.grapple_Solo.set_torque_reference_iq(torque_ref)
            
    # Function to publish data into topics
    def publish_data(self):
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
        
    # EXTERNAL FLAG MANAGEMENT  
    def GRASP_external_flags(self,msg):
        '''
        This function runs when we receive an external flag through the topic /GRASP_flags
        This means that this code does not run in a loop. It's only triggered with the flags, like an interruption. 
        We will first match the received flag and then perform the associated actions.
        '''
        flag = msg.data   
        match flag:
            # GRAPPLE related flags
            case 'GO_HOME':
                self.get_logger().info('Received command to home GRAPPLE mechanism.')
                self.grapple_Solo.set_control_mode(solo.ControlMode.TORQUE_MODE) 
                self.gra_motor_control_mode = "TORQUE"
                self.grapple_Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
                torque_ref = 2
                self.grapple_Solo.set_torque_reference_iq(torque_ref)
                self.grapple_state = "HOMING"

            case 'GO_OPEN':
                self.get_logger().info('Received command to open GRAPPLE mechanism.')
                self.grapple_Solo.set_speed_acceleration_value(100)
                self.grapple_Solo.set_speed_deceleration_value(100)
                self.grapple_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                self.gra_motor_control_mode = "POSITION"
                self.target_pos = -41500
                self.grapple_Solo.set_position_reference(self.target_pos)
                self.grapple_state = 'OPEN'

            case 'GO_SD':
                self.get_logger().info('Received command to go to soft dock position')
                self.grapple_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                self.gra_motor_control_mode = 'POSITION'
                self.target_pos = -39500
                self.grapple_Solo.set_position_reference(self.target_pos)
                self.grapple_state = 'SOFTDOCK'

            case 'GO_HD':
                self.get_logger().info('Received command to go to hard dock')
                self.grapple_Solo.set_control_mode(solo.ControlMode.TORQUE_MODE)
                self.gra_motor_control_mode = 'TORQUE'
                self.grapple_Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
                torque_ref = 2
                self.grapple_Solo.set_torque_reference_iq(torque_ref)
                self.grapple_state = 'HARDDOCK'

            case 'GO_CAPTURE':
                self.get_logger().info('Received command to capture RAFTI.')
                self.grapple_Solo.set_speed_acceleration_value(100)
                self.grapple_Solo.set_speed_deceleration_value(100)
                self.grapple_Solo.set_control_mode(solo.ControlMode.SPEED_MODE) 
                self.gra_motor_control_mode = "SPEED"
                self.grapple_Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
                speed_ref = 4750
                self.grapple_Solo.set_speed_reference(speed_ref)
                self.grapple_state = 'CAPTURING'

            case 'GO_RELEASE':
                self.get_logger().info('Received command to release RAFTI.')
                self.grapple_Solo.set_speed_limit(4750)
                self.grapple_Solo.set_speed_acceleration_value(10)
                self.grapple_Solo.set_speed_deceleration_value(10)
                self.target_pos  = -41500
                self.gra_motor_control_mode = "POSITION"
                self.grapple_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                self.grapple_Solo.set_position_reference(self.target_pos)
                self.grapple_state = 'RELEASE'

            case 'STOP':
                control_mode, error = self.grapple_Solo.get_control_mode()
                if control_mode == solo.ControlMode.POSITION_MODE or control_mode == solo.ControlMode.SPEED_MODE:
                    self.grapple_Solo.set_control_mode(solo.ControlMode.SPEED_MODE)
                    self.gra_motor_control_mode = "SPEED"
                    self.grapple_Solo.set_speed_reference(0)
                    self.get_logger().warning('')
                    print('Asked the motor to STOP by setting SPEED to zero.')
                    self.grapple_state = 'IDLE'
                else control_mode == solo.ControlMode.TORQUE_MODE:
                    self.grapple_Solo.set_control_mode(solo.ControlMode.TORQUE_MODE)
                    self.gra_motor_control_mode = "TORQUE"
                    self.get_logger().info(f"TORQUE MODE: Commanding the motor to stop by setting TORQUE to zero.")
                    self.grapple_Solo.set_torque_reference_iq(0)
                self.grapple_state = 'IDLE'

            case 'RESET':
                print('Resetting position to zero:')
                self.get_logger().info(f"Resetting position to zero")
                self.grapple_Solo.reset_position_to_zero()
                self.grapple_Solo.set_position_reference(0)
                self.grapple_state = 'IDLE'

                
    # STATE MACHINE CODE
    def GRASP_state_machine(self):
        '''This function manages the state machine of GRASP CONOPS.'''
        self.gra_motor_pos_ref, error = self.grapple_Solo.get_position_reference()
        self.gra_motor_pos, error     = self.grapple_Solo.get_position_counts_feedback()
        self.gra_motor_speed, error   = self.grapple_Solo.get_speed_feedback()
        self.gra_motor_current, error = self.grapple_Solo.get_quadrature_current_iq_feedback()

        # Checking GRAPPLE states and calling relevant code
        match self.grapple_state:

            case "IDLE":
                pass

            case "HOMING":
                current_threshold = 2
                if abs(self.gra_motor_current) > current_threshold:
                    self.get_logger().info('Current threshold for GRAPPLE homing reached.')
                    self.get_logger().info('Stopping motor by setting torque reference current to 0.')
                    self.grapple_Solo.set_control_mode(solo.ControlMode.TORQUE_MODE)
                    self.gra_motor_control_mode = "TORQUE"
                    self.grapple_Solo.set_torque_reference_iq(0)
                    time.sleep(0.5) # Short pause
                    self.gra_motor_pos_ref, error = self.grapple_Solo.get_position_reference()
                    self.gra_motor_pos, error     = self.grapple_Solo.get_position_counts_feedback()
                    self.gra_motor_speed, error   = self.grapple_Solo.get_speed_feedback()
                    self.gra_motor_current, error = self.grapple_Solo.get_quadrature_current_iq_feedback()
                    self.get_logger().info('Setting current position to 0 to complete HOMING')
                    self.grapple_Solo.reset_position_to_zero()
                    self.grapple_Solo.set_position_reference(0)
                    self.grapple_state = 'HOME'

            case "HOME":
                pass

            case "OPEN":
                pass

            case "SOFTDOCK":
                # Quick and dirty code to detect if something is going wrong in the soft docking procedure.
                current_threshold = 1.0
                if abs(self.gra_motor_current) > current_threshold:
                    self.get_logger().warning('Current threshold for SOFTDOCK exceeded. Back to OPEN and stopping motor.')
                    self.grapple_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                    self.grapple_Solo.set_position_reference(-41500)

            case "HARDDOCK":
                pass

            case "CAPTURING":
                current_threshold = 2
                capture_iq_threshold = 1.0
                capture_pos_threshold = -39500
                capture_error = False
                docking_done = False

                if (abs(self.gra_motor_pos) > abs(capture_pos_threshold)) and (abs(self.gra_motor_current) > capture_iq_threshold):
                    capture_error = True
                    self.get_logger().warning('Current threshold for SOFTDOCK exceeded. Back to OPEN and stopping motor.')
                    self.grapple_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                    self.grapple_Solo.set_position_reference(-41500)
                
                if abs(self.gra_motor_current) > current_threshold:
                    self.grapple_Solo.set_control_mode(solo.ControlMode.SPEED_MODE)
                    self.gra_motor_control_mode = "SPEED"
                    self.grapple_Solo.set_speed_reference(0)
                    self.get_logger().info('Current threshold for docking reached. Setting motor speed to zero.')
                    self.get_logger().info(f"GRAPPLE has reached a current limit! commanded the motor to stop by setting vel to zero.")
                    
                    time.sleep(2)
                    self.gra_motor_pos, error     = self.grapple_Solo.get_position_counts_feedback()
                    self.grapple_Solo.set_position_reference(self.gra_motor_pos)
                    self.gra_motor_pos_ref, error = self.grapple_Solo.get_position_reference()
                    self.gra_motor_speed, error   = self.grapple_Solo.get_speed_feedback()
                    self.gra_motor_current, error = self.grapple_Solo.get_quadrature_current_iq_feedback()
                    
                    self.grapple_state = 'HARD_DOCK'

            case "HARD_DOCK":
                pass
                
            case "RELEASE":
                self.get_logger().info('Updating GRASP state to RELEASE')
                
            case _:
                self.get_logger().error('Unidentified GRASP state request. Ignoring command.')
        
        self.publish_data()


def main():
    rclpy.init()
    GRASP_node = GRASPNode()
    rclpy.spin(GRASP_node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()