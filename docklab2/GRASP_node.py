#!/usr/bin/env python

# Importing ROS2 Python Client Libraries 
import rclpy
from rclpy.node import Node

# Import service type 
from std_srvs.srv import Trigger

# Import message type 
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int32

# Import Python libraries 
import serial
import time
import glob
from datetime import datetime
from enum import Enum, auto

# Import Solo Motor Controller Library
import SoloPy as solo # If solopy is not found in ROS2, need to run this before: 'export PYTHONPATH=/home/labpi/py_env/lib/python3.12/site-packages:$PYTHONPATH'

# Enum classes for Grapple and AVC states
class GrappleState(Enum):
    LAUNCH_LOCKED = auto()
    IDLE = auto()
    HOMING = auto()
    HOME = auto()
    SHIELDING = auto()
    SHIELDED = auto()
    OPENING = auto()
    OPEN = auto()
    SOFT_DOCKING = auto()
    SOFT_DOCK = auto() 
    HARD_DOCKING = auto()
    HARD_DOCK = auto()
    RELEASING = auto()
    RELEASE = auto()

class AVCState(Enum):
    LAUNCH_LOCKED = auto()
    IDLE = auto()
    HOMING = auto()
    HOME = auto()
    SHIELDING = auto()
    SHIELDED = auto()
    GOING_POS0 = auto()
    POS0 = auto()
    GOING_POS1 = auto()
    POS1 = auto()
    GOING_POS2 = auto()
    POS2 = auto()
    GOING_POS1p5 = auto()
    POS1p5 = auto()
    RETRACTING = auto()
    RETRACTED = auto()

    # Legacy state terms
    HOMED = auto()
    RETURNING = auto()
    

# Class definition for GRASP Node
class GRASPNode(Node):
    # Constructor, initialises motors by calling the init definitions.
    def __init__(self):

        # Set node name 
        super().__init__('GRASP_node')

        # Declaring parameters
        self.get_logger().debug('Declaring parameters for GRASP_node')
        self.declare_parameters(
            namespace='', 
            parameters=[
                ('solo_params.name', rclpy.Parameter.Type.STRING_ARRAY),
                ('solo_params.address', rclpy.Parameter.Type.INTEGER_ARRAY),
                ('solo_params.baudrate', rclpy.Parameter.Type.INTEGER_ARRAY),
                ('solo_params.pwm_frequency_khz', rclpy.Parameter.Type.INTEGER_ARRAY),
                ('solo_params.current_limit', rclpy.Parameter.Type.INTEGER_ARRAY),
                ('solo_params.motor_poles_counts', rclpy.Parameter.Type.INTEGER_ARRAY),
                ('solo_params.command_mode', rclpy.Parameter.Type.STRING_ARRAY),
                ('solo_params.motor_type', rclpy.Parameter.Type.STRING_ARRAY),
                ('solo_params.feedback_control_mode', rclpy.Parameter.Type.STRING_ARRAY),
                ('solo_params.current_controller_kp', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('solo_params.current_controller_ki', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('solo_params.speed_controller_kp', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('solo_params.speed_controller_ki', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('solo_params.position_controller_kp', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('solo_params.position_controller_ki', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('solo_params.control_mode', rclpy.Parameter.Type.STRING_ARRAY),
                ('solo_params.speed_limit', rclpy.Parameter.Type.INTEGER_ARRAY),
                ('solo_params.speed_acceleration', rclpy.Parameter.Type.INTEGER_ARRAY),
                ('solo_params.speed_deceleration', rclpy.Parameter.Type.INTEGER_ARRAY),
                ('solo_params.logger_level', rclpy.Parameter.Type.INTEGER_ARRAY),

                ('feedback_params.frequency', rclpy.Parameter.Type.DOUBLE),

                ('state_machine_params.frequency', rclpy.Parameter.Type.DOUBLE)
            ])
        
        # Find available serial ports
        self.available_ports = self.find_available_ports()
        
        # Initialize motors
        self.get_logger().debug('Initializing motors')

        self.grapple_Solo = self.motor_init('grapple')
        self.grapple_state = GrappleState.IDLE
        self.get_logger().info("Grapple set to IDLE")

        self.avc_Solo = self.motor_init('avc')
        self.avc_state = AVCState.IDLE
        self.get_logger().info('AVC set to IDLE')
        
        # Set up a function that constantly monitors the state machine
        self.state_machine_frequency = self.get_parameter('state_machine_params.frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / self.state_machine_frequency, self.GRASP_state_machine)

        # Set up a function that publishes data at a set frequency
        self.feedback_frequency = self.get_parameter('feedback_params.frequency').get_parameter_value().double_value
        self.feedback_timer = self.create_timer(1.0 / self.feedback_frequency, self.publish_data)
        
        # Add GRASP state subscribers
        self.subscription           = self.create_subscription(String, 'GRASP_flags',                self.GRASP_external_flags, 10) #QoS arbitrarily set at 10
        self.subscription           = self.create_subscription(Float64,'grapple_motor/position_cmd', self.grapple_motor_position_control,10)
        self.subscription           = self.create_subscription(Float64,'grapple_motor/velocity_cmd', self.grapple_motor_speed_control,10)
        self.subscription           = self.create_subscription(Float64,'grapple_motor/torque_cmd',   self.grapple_motor_torque_control,10)
        self.subscription           = self.create_subscription(Float64,'avc_motor/position_cmd',     self.avc_motor_position_control,10)
        self.subscription           = self.create_subscription(Float64,'avc_motor/velocity_cmd',     self.avc_motor_speed_control,10)
        self.subscription           = self.create_subscription(Float64,'avc_motor/torque_cmd',       self.avc_motor_torque_control,10)
        self.subscription # prevent unused variable error
        
        # Add GRASP publishers
        self.pub_gra_motor_feedback = self.create_publisher(String,  'gra_motor_feedback', 10)
        self.pub_gra_motor_curr_iq  = self.create_publisher(Float64, 'gra_motor_current_iq', 10)
        self.pub_gra_motor_pos      = self.create_publisher(Int32,   'gra_motor_pos', 10)
        self.pub_gra_motor_vel      = self.create_publisher(Int32,   'gra_motor_vel', 10)
        self.pub_avc_motor_feedback = self.create_publisher(String,  'avc_motor_feedback', 10)
        self.pub_avc_motor_curr_iq  = self.create_publisher(Float64, 'avc_motor_current_iq', 10)
        self.pub_avc_motor_pos      = self.create_publisher(Int32,   'avc_motor_pos', 10)
        self.pub_avc_motor_vel      = self.create_publisher(Int32,   'avc_motor_vel', 10)
        
        self.get_logger().info('GRASP_node initiated with a periodic state machine, AVC and grapple state set to IDLE.') # Don't we want to set this to IDLE first by default?

    # Function to find available serial ports
    def find_available_ports(self):
        # Search for available serial devices. 
        self.get_logger().info('Running a search for available serial devices')
        ports_found = 0
        while ports_found == 0:
            ports_searched = glob.glob('/dev/ttyACM[0-9]*')
            ports_found = len(ports_searched)
            if ports_found == 0:
                self.get_logger().info('No serial devices found. Awaiting connection.')
            time.sleep(1)
        available_ports = []
        for port in ports_searched:
            try:
                s = serial.Serial(port)
                s.close()
                available_ports.append(str(port))
                self.get_logger().info(f"Found a device active on: {port}")
            except:
                pass
        return available_ports

    # Function for initialising motor
    def motor_init(self, name):
        # Get parameters for initialising motor 
        self.get_logger().debug(f'Initializing {name} motor driver')
        # Get the parameter index for the motor name
        idx = self.get_parameter(f'solo_params.name').get_parameter_value().string_array_value.index(name)
        address = self.get_parameter(f'solo_params.address').get_parameter_value().integer_array_value[idx]
        baudrate = self.get_parameter(f'solo_params.baudrate').get_parameter_value().integer_array_value[idx]
        pwm_frequency_khz = self.get_parameter(f'solo_params.pwm_frequency_khz').get_parameter_value().integer_array_value[idx]
        current_limit = self.get_parameter(f'solo_params.current_limit').get_parameter_value().integer_array_value[idx]
        motor_poles_counts = self.get_parameter(f'solo_params.motor_poles_counts').get_parameter_value().integer_array_value[idx]
        command_mode = self.get_parameter(f'solo_params.command_mode').get_parameter_value().string_array_value[idx]
        motor_type = self.get_parameter(f'solo_params.motor_type').get_parameter_value().string_array_value[idx]
        feedback_control_mode = self.get_parameter(f'solo_params.feedback_control_mode').get_parameter_value().string_array_value[idx]
        current_controller_kp = self.get_parameter(f'solo_params.current_controller_kp').get_parameter_value().double_array_value[idx]
        current_controller_ki = self.get_parameter(f'solo_params.current_controller_ki').get_parameter_value().double_array_value[idx]
        speed_controller_kp = self.get_parameter(f'solo_params.speed_controller_kp').get_parameter_value().double_array_value[idx]
        speed_controller_ki = self.get_parameter(f'solo_params.speed_controller_ki').get_parameter_value().double_array_value[idx]
        position_controller_kp = self.get_parameter(f'solo_params.position_controller_kp').get_parameter_value().double_array_value[idx]
        position_controller_ki = self.get_parameter(f'solo_params.position_controller_ki').get_parameter_value().double_array_value[idx]
        control_mode = self.get_parameter(f'solo_params.control_mode').get_parameter_value().string_array_value[idx]
        speed_limit = self.get_parameter(f'solo_params.speed_limit').get_parameter_value().integer_array_value[idx]
        speed_acceleration = self.get_parameter(f'solo_params.speed_acceleration').get_parameter_value().integer_array_value[idx]
        speed_deceleration = self.get_parameter(f'solo_params.speed_deceleration').get_parameter_value().integer_array_value[idx]
        logger_level = self.get_parameter(f'solo_params.logger_level').get_parameter_value().integer_array_value[idx]

        if baudrate == 937500:
            soloBaudrate = solo.UartBaudRate.RATE_937500
        else: 
            self.get_logger().error(f"Unrecognised baudrate parsed as {name} motor driver parameter")
        if command_mode == 'DIGITAL': 
            command_mode = solo.CommandMode.DIGITAL
        else: 
            self.get_logger().error(f"Unrecognised command mode parsed as {name} motor driver parameter")
        if motor_type == 'BLDC_PMSM':
            motor_type = solo.MotorType.BLDC_PMSM
        else: 
            self.get_logger().error(f"Unrecognised motor type parsed as {name} motor driver parameter")
        if feedback_control_mode == 'HALL_SENSORS':
            feedback_control_mode = solo.FeedbackControlMode.HALL_SENSORS
        else:
            self.get_logger().error(f"Unrecognised feedback control mode parsed as {name} motor driver parameter")
        if control_mode == 'POSITION_MODE':
            control_mode = solo.ControlMode.POSITION_MODE
        else: 
            self.get_logger().error(f"Unrecognised control mode parsed as {name} motor driver parameter")

        # Connecting to correct motor driver
        connection_successful = False
        while connection_successful == False:
            for port in self.available_ports:
                self.get_logger().debug(f"Attempting to connect to {name} motor driver over {port}")
                Solo = solo.SoloMotorControllerUart(port=port, baudrate=soloBaudrate, address=address, loggerLevel=logger_level)
                read_address = Solo.get_device_address()[0]
                self.get_logger().debug(f"{port} reads back motor address as {read_address}")
                if read_address == address:
                    self.get_logger().info(f"Successfully connected to {name} motor driver over {port}")
                    connection_successful = True
                    self.available_ports.remove(port)
                    break
                else:
                    self.get_logger().debug(f"Could not find {name} motor driver over {port}")
                    self.get_logger().debug(f"Disconnecting from {port}")
                    Solo.serial_close()
                    connection_successful = False

        # Setting up motor driver
        # Reset initial position to zero
        self.get_logger().info(f"Resetting {name} position to zero")
        Solo.reset_position_to_zero()

        # Applying motor parameters
        self.get_logger().debug(f"Applying {name} motor parameters")
        Solo.set_output_pwm_frequency_khz(pwm_frequency_khz)              # Desired switching or PWM frequency at output
        Solo.set_current_limit(current_limit)                          # Current limit of the motor
        Solo.set_motor_poles_counts(motor_poles_counts)                     # Motor's number of poles
        Solo.set_command_mode(command_mode) 
        Solo.set_motor_type(motor_type)
        Solo.set_feedback_control_mode(feedback_control_mode)
        Solo.set_current_controller_kp(current_controller_kp)                # Current controller Kp
        Solo.set_current_controller_ki(current_controller_ki)              # Current controller Ki
        Solo.set_speed_controller_kp(speed_controller_kp)                  # Speed controller Kp
        Solo.set_speed_controller_ki(speed_controller_ki)                # Speed controller Ki
        Solo.set_position_controller_kp(position_controller_kp)                # Position controller Kp
        Solo.set_position_controller_ki(position_controller_ki)                 # Position controller Ki
        Solo.set_control_mode(control_mode)
        Solo.set_speed_limit(speed_limit)                         # Desired Speed Limit[RPM].
        Solo.set_speed_acceleration_value(speed_acceleration)             # Speed acceleration limit[RPM].
        Solo.set_speed_deceleration_value(speed_deceleration)             # Speed deceleration limit[RPM].

        # Log out set values
        self.get_logger().debug(f"""
        {name} motor driver read parameters:\n
        Board Temperature: {str(Solo.get_board_temperature()[0])} degrees\n
        The position controller gains for the {name} motor are:\n
        Kp = {str(Solo.get_position_controller_kp()[0] )}\n
        Ki = {str(Solo.get_position_controller_ki()[0] )}\n
        The velocity controller gains are:\n
        Kp = {str(Solo.get_speed_controller_kp()[0])}\n
        Ki = {str(Solo.get_speed_controller_ki()[0])}\n
        The current controller gains are:\n
        Kp = {str(Solo.get_current_controller_kp()[0])}\n
        Ki = {str(Solo.get_current_controller_ki()[0])}\n
        Control mode is: {str(Solo.get_control_mode()[0])}\n
        Speed Limit is: {str(Solo.get_speed_limit()[0])}\n
        Current Limit is: {str(Solo.get_current_limit()[0])}\n
        Speed acceleration limit is: {str(Solo.get_speed_acceleration_value()[0])}\n
        """)

        return Solo

    def motor_position_control(self, Solo, position_reference):
        # This a function to request a specific position to the motor.
        Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
        self.get_logger().debug(f"Changing motor to position mode control and setting position reference {position_reference}")
        Solo.set_position_reference(position_reference)
        return

    def motor_speed_control(self, Solo, speed_reference):
        # This a function to request a specific speed to the motor.
        Solo.set_control_mode(solo.ControlMode.SPEED_MODE)
        if speed_reference < 0:
            Solo.set_motor_direction(solo.Direction.CLOCKWISE)
            speed_reference = abs(speed_reference)
            self.get_logger().debug(f"Changing motor to speed mode control and setting speed reference {speed_reference} in CLOCKWISE direction")
        else:
            Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
            speed_reference = abs(speed_reference)
            self.get_logger().debug(f"Changing motor to speed mode control and setting speed reference {speed_reference} in COUNTERCLOCKWISE direction")
        Solo.set_speed_reference(speed_reference)
        return

    def motor_torque_control(self, Solo, torque_reference):
        # This a function to request a specific torque to the motor.
        Solo.set_control_mode(solo.ControlMode.TORQUE_MODE)
        if torque_reference < 0:
            Solo.set_motor_direction(solo.Direction.CLOCKWISE)
            torque_reference = abs(torque_reference)
            self.get_logger().debug(f"Changing motor to torque mode control and setting torque reference {torque_reference} in CLOCKWISE direction")
        else:
            Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
            torque_reference = abs(torque_reference)
            self.get_logger().debug(f"Changing motor to torque mode control and setting torque reference {torque_reference} in COUNTERCLOCKWISE direction")
        Solo.set_torque_reference_iq(torque_reference)
        return
    
    # Function to publish data into topics
    def publish_data(self):
        # This function publish the data we want to record for external analysis
        
        # ----------------- Grapple data ----------------------------------
        msg = String()
        msg.data = f"State: {self.grapple_state.name}, Pos ref: {self.gra_motor_pos_ref}, Pos counts: {self.gra_motor_pos},Speed: {self.gra_motor_speed},Current Iq: {self.gra_motor_current}"
        self.pub_gra_motor_feedback.publish(msg)

        #Position Feedback
        grapple_motor_pos_msg = Int32()
        grapple_motor_pos_msg.data = self.gra_motor_pos
        self.pub_gra_motor_pos.publish(grapple_motor_pos_msg)
        
        #Velocity Feedback
        grapple_motor_vel_msg = Int32()
        grapple_motor_vel_msg.data = self.gra_motor_speed
        self.pub_gra_motor_vel.publish(grapple_motor_vel_msg)
        
        #Current Phase Iq Feedback
        grapple_current_iq_msg = Float64()
        grapple_current_iq_msg.data = self.gra_motor_current
        self.pub_gra_motor_curr_iq.publish(grapple_current_iq_msg)
        
        # ----------------- AVC data -----------------------------------
        avc_msg = String()
        avc_msg.data = f"State: {self.avc_state.name}, Pos ref: {self.avc_motor_pos_ref}, Pos counts: {self.avc_motor_pos},Speed: {self.avc_motor_speed},Current Iq: {self.avc_motor_current}"
        self.pub_avc_motor_feedback.publish(avc_msg)

        #Position Feedback
        avc_motor_pos_msg = Int32()
        avc_motor_pos_msg.data = self.avc_motor_pos
        self.pub_avc_motor_pos.publish(avc_motor_pos_msg)
        
        #Velocity Feedback
        avc_motor_vel_msg = Int32()
        avc_motor_vel_msg.data = self.avc_motor_speed
        self.pub_avc_motor_vel.publish(avc_motor_vel_msg)
        
        #Current Phase Iq Feedback
        avc_current_iq_msg = Float64()
        avc_current_iq_msg.data = self.avc_motor_current
        self.pub_avc_motor_curr_iq.publish(avc_current_iq_msg)
    
    # Debugging functions to interact directly with grapple motor for position, speed and torque control
    def grapple_motor_position_control(self, msg):
        self.get_logger().info(f"Received grapple position command: {msg.data}")
        self.motor_position_control(self.grapple_Solo, msg.data)
        return

    def grapple_motor_speed_control(self, msg):
        self.get_logger().info(f"Received grapple speed command: {msg.data}")
        self.motor_speed_control(self.grapple_Solo, msg.data)
        return
    
    def grapple_motor_torque_control(self, msg):
        self.get_logger().info(f"Received grapple torque command: {msg.data}")
        self.motor_torque_control(self.grapple_Solo, msg.data)
        return
        
    # Debugging functions to interact directly with AVC motor for position, speed and torque control
    def avc_motor_position_control(self, msg):
        self.get_logger().info(f"Received AVC position command: {msg.data}")
        self.motor_position_control(self.avc_Solo, msg.data)
        return
    
    def avc_motor_speed_control(self, msg):
        self.get_logger().info(f"Received AVC speed command: {msg.data}")
        self.motor_speed_control(self.avc_Solo, msg.data)
        return
    
    def avc_motor_torque_control(self, msg):
        self.get_logger().info(f"Received AVC torque command: {msg.data}")
        self.motor_torque_control(self.avc_Solo, msg.data)
        return
        
    # EXTERNAL FLAG MANAGEMENT  
    def GRASP_external_flags(self, msg):
        '''This function runs when we receive an external flag through the topic /GRASP_flags
           This means that this code does not run in a loop. It's only triggered with the flags, like an interruption. 
           We will first match the received flag and then perform the associated actions.'''
        flag = msg.data   
        match flag:
            # Grapple related flags
            case 'GO_HOME':
                # Send confirmation message
                self.get_logger().info('Received command: GO_HOME.')
                # Set new state as 'HOMING' for the state machine.
                self.grapple_state = GrappleState.HOMING
                self.get_logger().info('Changed grapple state to HOMING.')
            case 'GO_SHIELDED':  # PROBE SHIELDING NOT CURRENTLY IMPLEMENTED
                # Send confirmation message
                self.get_logger().info('Received command: GO_SHIELDED.')
                # Check for previous state. Cannot transition to SHIELDED without previous state being HOME.
                if self.grapple_state != GrappleState.HOME:
                    self.get_logger().warning("Failed to change grapple mode to 'SHIELDING' because previous state isn't 'HOME'. ")
                    return
                self.grapple_state = GrappleState.SHIELDING
                self.get_logger().info('Changed grapple state to SHIELDING.')
            case 'GO_OPEN':
                # Send confirmation message
                self.get_logger().info('Received command: GO_OPEN.')
                # Check for previous state. Cannot transition to OPEN without previous state being HOME or SHIELDED.
                if self.grapple_state != GrappleState.HOME and self.grapple_state != GrappleState.SHIELDED:
                    self.get_logger().warning("Failed to change grapple mode to 'OPENING' because previous state isn't 'HOME' or 'SHIELDED'. ")
                    return
                self.grapple_state = GrappleState.OPENING
                self.get_logger().info('Changed grapple state to OPENING.')
            case 'GO_DOCK':
                # Send confirmation message
                self.get_logger().info('Received command: GO_DOCK. Grapple will execute soft dock followed by hard dock.')
                # Check for previous state. Cannot transition to SOFT_DOCKING without previous state being OPEN
                if self.grapple_state != GrappleState.OPEN:
                    self.get_logger().warning("Failed to change grapple mode to 'SOFT_DOCKING' because previous state isn't 'OPEN'. ")
                    return
                self.grapple_state = GrappleState.SOFT_DOCKING
                self.get_logger().info('Changed grapple state to SOFT_DOCKING.')
            case 'GO_RELEASE':
                # Send confirmation message
                self.get_logger().info('Received command: GO_RELEASE.')
                # Check for previous state. Cannot transition to RELEASING without previous state being HARD_DOCK
                if self.grapple_state != GrappleState.HARD_DOCK:
                    self.get_logger().warning("Failed to change grapple mode to 'RELEASING' because previous state isn't 'HARD_DOCK'. ")
                    return
                self.grapple_state = GrappleState.RELEASING
                self.get_logger().info('Changed grapple state to RELEASING.')

            # Testing and debugging flags
            case 'STOP':
                self.get_logger().warning('Received command: STOP. Stopping grapple motor immediately.')
                self.grapple_Solo.emergency_stop()                
                self.grapple_state = GrappleState.IDLE
                self.get_logger().info('Changed grapple state to IDLE.')

            # AVC related flags
            case 'GO_AVC_HOMING':
                # Send confirmation message
                self.get_logger().info('Received command to home AVC mechanism.')
                # Set control mode to torque. AVC homing done in torque mode.
                self.avc_Solo.set_control_mode(solo.ControlMode.TORQUE_MODE) 
                # Set motor spin direction. We want to retract AVC carriage, which is done by moving the AVC motor counterclockwise.
                self.avc_Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
                # Set torque reference current. Homing tested to work at 0.08A. Do NOT increase without consulting PM.
                torque_ref = 0.08
                self.avc_Solo.set_torque_reference_iq(torque_ref)
                # Set new AVC state as 'HOME' for the state machine.
                self.avc_state = AVCState.HOME

            case 'GO_POS1':
                # Send confirmation message
                self.get_logger().info('Received command to send AVC to POS1 (leak check).')
                # Check for previous state. Cannot proceed to POS1 or beyond without HOMING.
                if self.avc_state != AVCState.HOMED:
                    self.get_logger().warning("Failed to proceed to POS1 as AVC has not been homed.")
                    return
                # Set control mode to position. AVC operation relies on knowing position data.
                self.avc_Solo.set_speed_acceleration_value(100)            # Speed acceleration limit[RPM].
                self.avc_Solo.set_speed_deceleration_value(100)            # Speed deceleration limit[RPM].
                self.avc_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                # Set target position. After performing HOME, POS1 should be at approximately -2790 QP. This includes a 0.6mm increase to account for test results.
                self.target_pos = -2790
                self.get_logger().info(f"Commanding AVC motor to target position: {self.target_pos}")
                self.avc_Solo.set_position_reference(self.target_pos)
                self.avc_state = AVCState.POS1
            case 'GO_POS1.5':
                # Send confirmation message
                self.get_logger().info('Received command to send AVC to POS1.5 (interstitial venting)')
                # Not checking for previous state right now. Will need to perform tests with caution.
                self.avc_Solo.set_speed_acceleration_value(100)            # Speed acceleration limit[RPM].
                self.avc_Solo.set_speed_deceleration_value(100)            # Speed deceleration limit[RPM].
                self.avc_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                # Set target position. POS1.5 should be at approximately -6400 QP. This is from testing conducted by SPF and PM by blowing air through pipes.
                self.target_pos = -8700 # Found this value on 09/07/2025 testing with new valve core installed.
                self.get_logger().info(f"Commanding AVC motor to target position: {self.target_pos}")
                self.avc_Solo.set_position_reference(self.target_pos)
                self.avc_state = AVCState.POS1p5
            case 'GO_POS2':
                # Send confirmation message
                self.get_logger().info('Received command to send AVC to POS2 (fluid transfer)')
                # Not checking for previous state right now. Will need to perform tests with caution.
                self.avc_Solo.set_speed_acceleration_value(100)            # Speed acceleration limit[RPM].
                self.avc_Solo.set_speed_deceleration_value(100)            # Speed deceleration limit[RPM].
                self.avc_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                # Set target position. POS2 should be at approximately -8546 QP. For the purposes of testing, going -7500 to stay on the safer side as its still open.
                self.target_pos = -10000 # Found this value on 09/07/2025 testing with new valve core installed
                self.get_logger().info(f"Commanding AVC motor to target position: {self.target_pos}")
                self.avc_Solo.set_position_reference(self.target_pos)
                self.avc_state = AVCState.POS2
            case 'AVC_RETURN':
                # Send confirmation message
                self.get_logger().info('Received command to send AVC back home.')
                # Not checking for previous state right now. Will need to perform tests with caution.
                self.avc_Solo.set_speed_acceleration_value(100)
                self.avc_Solo.set_speed_deceleration_value(100)
                self.avc_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                # Set target position. Venting is essentially a return to POS1.5, but we want to go further to ensure ball closed but poppet open. Tested at -4595.
                self.target_pos = 0
                self.get_logger().info(f"Commanding AVC motor to target position: {self.target_pos}")
                self.avc_Solo.set_position_reference(self.target_pos)
                self.avc_state = AVCState.RETURNING
            case 'AVC_UNSTUCKIFY':
                # Send confirmation message
                self.get_logger().info('Good job on getting the AVC stuck.')
                self.avc_Solo.set_speed_acceleration_value(100)
                self.avc_Solo.set_speed_deceleration_value(100)
                self.avc_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
                # Set target position
                self.current_pos = self.avc_Solo.get_position_counts_feedback()
                self.target_pos = 2000
                self.get_logger().info(f"Commanding AVC motor to target position: {self.target_pos}")

                
    # STATE MACHINE CODE
    def GRASP_state_machine(self):

        # Get the current state from Solo controllers
        self.gra_motor_pos_ref, error = self.grapple_Solo.get_position_reference()
        self.gra_motor_speed_ref, error = self.grapple_Solo.get_speed_reference()
        self.gra_motor_torque_ref, error = self.grapple_Solo.get_torque_reference_iq()
        self.gra_motor_control_mode, error = self.grapple_Solo.get_control_mode()
        self.gra_motor_pos, error     = self.grapple_Solo.get_position_counts_feedback()
        self.gra_motor_speed, error   = self.grapple_Solo.get_speed_feedback()
        self.gra_motor_current, error = self.grapple_Solo.get_quadrature_current_iq_feedback()

        self.avc_motor_pos_ref, error = self.avc_Solo.get_position_reference()
        self.avc_motor_speed_ref, error = self.avc_Solo.get_speed_reference()
        self.avc_motor_torque_ref, error = self.avc_Solo.get_torque_reference_iq()
        self.avc_motor_control_mode, error = self.avc_Solo.get_control_mode()
        self.avc_motor_pos, error     = self.avc_Solo.get_position_counts_feedback()
        self.avc_motor_speed, error   = self.avc_Solo.get_speed_feedback()
        self.avc_motor_current, error = self.avc_Solo.get_quadrature_current_iq_feedback()

        # Checking grapple states and calling relevant code
        match self.grapple_state:
            case GrappleState.LAUNCH_LOCKED:
                '''Passive state. Grapple mechanisms locked for launch. Awaiting external flags.'''
                # Launch locking not currently implemented. Placeholder for future code.
                pass

            case GrappleState.IDLE:
                '''Default state. No action, awaiting external flags.'''
                pass

            case GrappleState.HOMING:
                '''Active state. Grapple mechanisms moving towards fully retracted position.'''
                target_iq = 2 # [A] Configuration parameter to be added to config file at later date.
                if self.gra_motor_torque_ref != target_iq or self.gra_motor_control_mode != solo.ControlMode.TORQUE_MODE:
                    self.motor_torque_control(self.grapple_Solo, target_iq)
                # State exit condition
                if abs(self.gra_motor_current) >= target_iq:
                    self.get_logger().debug('Current target for grapple homing reached.')
                    self.motor_torque_control(self.grapple_Solo, 0)
                    time.sleep(0.5) # Short pause for system stabilisation to occur
                    self.get_logger().debug('Setting current position to 0 to complete HOMING')
                    # Resetting position to 0 QP. Setting state to HOME.
                    self.grapple_Solo.reset_position_to_zero()
                    self.grapple_Solo.set_position_reference(0)
                    self.grapple_state = GrappleState.HOME
                    self.get_logger().info('Changed grapple state to HOME.')

            case GrappleState.HOME:
                '''Passive state. Grapple mechanisms have retracted to home position. Awaiting external flags.'''
                # Setting motor torque to 0 to stop motor
                self.motor_torque_control(self.grapple_Solo, 0)
                

            case GrappleState.SHIELDING:
                '''Active state. Grapple mechanisms moving to shield AVC probes.'''
                # Probe shielding not currently implemented. Placeholder for future code.
                pass

            case GrappleState.SHIELDED:
                '''Passive state. Grapple mechanisms have shielded AVC probes. Awaiting external flags.'''
                # Probe shielding not currently implemented. Placeholder for future code.
                pass

            case GrappleState.OPENING:
                '''Active state. Grapple mechanisms moving to open position.'''
                target_pos = -41000 # [QP] Configuration parameter to be added to config file at later date.
                if self.gra_motor_pos_ref != target_pos or self.gra_motor_control_mode != solo.ControlMode.POSITION_MODE:
                    self.motor_position_control(self.grapple_Solo, target_pos)
                # State exit condition
                if self.gra_motor_pos <= target_pos:
                    self.get_logger().debug('Position target for grapple opening reached.')
                    self.grapple_state = GrappleState.OPEN
                    self.get_logger().info('Changed grapple state to OPEN.')


            case GrappleState.OPEN:
                '''Passive state. Grapple mechanisms have opened to allow docking. Awaiting external flags.'''
                # Setting motor torque to 0 to stop motor
                self.motor_torque_control(self.grapple_Solo, 0)

            case GrappleState.SOFT_DOCKING:
                '''Active state. Grapple mechanisms moving to soft dock position.'''
                target_speed = 4750 # [QPS] Configuration parameter to be added to config file at later date.
                if self.gra_motor_speed_ref != target_speed or self.gra_motor_control_mode != solo.ControlMode.SPEED_MODE:
                    self.motor_speed_control(self.grapple_Solo, target_speed)
                # State exit condition
                if self.gra_motor_pos >= -10000: # [QP] Configuration parameter to be added to config file at later date. Need to find what this value is from Rhys and Palash
                    self.get_logger().debug('Position target for grapple soft docking reached.')
                    self.grapple_state = GrappleState.SOFT_DOCK
                    self.get_logger().info('Changed grapple state to SOFT_DOCK.')

            case GrappleState.SOFT_DOCK:
                '''Active state. Grapple mechanisms have reached soft dock position. Chance for error checking and correction with possible abort.'''
                # Abort not currently implemented. Placeholder for future code.
                self.grapple_state = GrappleState.HARD_DOCKING
                self.get_logger().info('Changed grapple state to HARD_DOCKING.')
                
            case GrappleState.HARD_DOCKING:
                '''Active state. Grapple mechanisms moving to hard dock position.'''
                target_iq = 2 # [A] Configuration parameter to be added to config file at later date.
                if self.gra_motor_torque_ref != target_iq or self.gra_motor_control_mode != solo.ControlMode.TORQUE_MODE:
                    self.motor_torque_control(self.grapple_Solo, target_iq)
                # State exit condition
                if abs(self.gra_motor_current) >= target_iq:
                    self.get_logger().debug('Current target for grapple hard docking reached.')
                    self.grapple_state = GrappleState.HARD_DOCK
                    self.get_logger().info('Changed grapple state to HARD_DOCK.')

            case GrappleState.HARD_DOCK:
                '''Passive state. Grapple mechanisms have reached hard dock position. Awaiting external flags.'''
                # Setting motor torque to 0 to stop motor
                self.motor_torque_control(self.grapple_Solo, 0) # May need to apply torque to maintain force. To be tested.

            case GrappleState.RELEASING:
                '''Active state. Grapple mechanisms moving to release docked object.'''
                target_pos = -41000 # [QP] Configuration parameter to be added to config file
                if self.gra_motor_pos_ref != target_pos or self.gra_motor_control_mode != solo.ControlMode.POSITION_MODE:
                    self.motor_position_control(self.grapple_Solo, target_pos)
                # State exit condition
                if self.gra_motor_pos <= target_pos:
                    self.get_logger().debug('Position target for grapple releasing reached.')
                    self.grapple_state = GrappleState.RELEASE
                    self.get_logger().info('Changed grapple state to RELEASE.')

            case GrappleState.RELEASE:
                '''Passive state. Grapple mechanisms have released docked object. Awaiting external flags.'''
                # Setting motor torque to 0 to stop motor
                self.motor_torque_control(self.grapple_Solo, 0)

            case _: # Catch invalid command 
                self.get_logger().error('Unknown GRASP state received')

        # Checking AVC states and calling relevant code. Should be extensible to multiple AVCs.
        match self.avc_state:
            case AVCState.HOME:
                '''Active state. AVC mechanisms moving towards fully retracted position.'''
                if abs(self.avc_motor_speed) == 0:
                    self.get_logger().info('AVC mechanism hit 0 RPM. Perform visual check to confirm homing.')
                    self.avc_Solo.set_control_mode(solo.ControlMode.TORQUE_MODE)
                    self.avc_Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
                    self.get_logger().info('Setting AVC torque reference to 0. Resetting home position.')
                    self.avc_Solo.set_torque_reference_iq(0)
                    self.avc_Solo.reset_position_to_zero()
                    self.avc_Solo.set_position_reference(0)
                    self.avc_state = AVCState.HOMED
                else:
                    pass
            case AVCState.HOMED:
                self.avc_Solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
                self.avc_Solo.set_torque_reference_iq(0)
                self.avc_Solo.reset_position_to_zero()
                self.avc_Solo.set_position_reference(0)
            case AVCState.POS1:
                pass
            case AVCState.POS1p5:
                pass
            case AVCState.POS2:
                pass
            case AVCState.RETURNING:
                if self.avc_motor_pos > -5:
                    self.get_logger().info('AVC returned to home position. Changing state to AVC_HOME.')
                    self.avc_state = AVCState.HOMED
        
def main():
    rclpy.init()
    GRASP_node = GRASPNode()
    rclpy.spin(GRASP_node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()