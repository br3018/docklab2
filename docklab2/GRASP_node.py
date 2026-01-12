#!/usr/bin/env python

# Importing ROS2 Python Client Libraries 
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

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

# Enum classes for GRASP modes
class GRASPMode(Enum):
    """Enumeration of possible GRASP operation modes."""
    ERROR = auto()
    UNCONTROLLED = auto()
    LAUNCH_LOCKED = auto()
    GRAPPLE_HOME = auto()
    AVC_A_HOME = auto()
    AVC_B_HOME = auto()
    FREE_FLIGHT = auto()
    READY = auto()
    SOFT_DOCK = auto()
    HARD_DOCK = auto()
    CLEARANCE = auto()
    AVC_A_POS1 = auto()
    AVC_A_POS1p5 = auto()
    AVC_A_POS2 = auto()
    AVC_A_RETRACT = auto()
    AVC_B_POS1 = auto()
    AVC_B_POS1p5 = auto()
    AVC_B_POS2 = auto()
    AVC_B_RETRACT = auto()
    RELEASE = auto()
    HEATER_ON = auto()
    HEATER_OFF = auto()
    
# Enum classes for GRASP commands
class GRASPCommand(Enum):
    """Enumeration of possible GRASP commands."""
    RESET = auto()
    HOME = auto()
    LAUNCH_LOCK = auto()
    FREE_FLIGHT = auto()
    READY = auto()
    TRIGGER = auto()
    CLEARANCE = auto()
    AVC_A_POS1 = auto()
    AVC_A_POS1p5 = auto()
    AVC_A_POS2 = auto()
    AVC_A_RETRACT = auto()
    AVC_B_POS1 = auto()
    AVC_B_POS1p5 = auto()
    AVC_B_POS2 = auto()
    AVC_B_RETRACT = auto()
    RELEASE = auto()
    
# Enum classes for Grapple and AVC states
class GrappleState(Enum):
    """Enumeration of possible states for the grapple mechanism."""
    ERROR = auto()
    UNCONTROLLED = auto()
    HOME = auto()
    FREE_FLIGHT = auto()
    OPEN = auto()
    SOFT_DOCK = auto()
    HARD_DOCK = auto()
    CLEARANCE = auto()
    PAUSED = auto()

class AVCState(Enum):
    """Enumeration of possible states for the AVC mechanism."""
    ERROR = auto()
    UNCONTROLLED = auto()
    LAUNCH_LOCK = auto()
    HOME = auto()
    POS1 = auto()
    POS1p5 = auto()
    POS2 = auto()

# Class definition for GRASP Node
class GRASPNode(Node):
    """
    ROS2 Node for controlling GRASP and AVC mechanisms via Solo motor controllers.
    Handles state machines, motor initialization, feedback publishing, and command subscriptions.
    """
    def __init__(self):

        # Set node name 
        super().__init__('GRASP_node')
        
        # Initializing GRASP mode
        self.GRASP_mode = GRASPMode.UNCONTROLLED
        
        # Initializing GRASP command
        self.GRASP_command = GRASPCommand.RESET

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
        
        # Grapple Mechanism
        try:
            self.grapple_Solo = self.motor_init('grapple')
            self.grapple_state = GrappleState.UNCONTROLLED
            self.grapple_homed = False
            self.get_logger().info("Grapple state set to UNCONTROLLED")
        except Exception as e:
            self.get_logger().warning(f"Failed to initialize grapple motor: {e}")
            self.grapple_state = GrappleState.ERROR
            self.grapple_Solo = None
            self.grapple_homed = False
            self.get_logger().info("Grapple state set to ERROR")

        # AVC A Mechanism
        try:
            self.avc_a_Solo = self.motor_init('avc_a')
            self.avc_a_state = AVCState.UNCONTROLLED
            self.avc_a_homed = False
            self.get_logger().info('AVC A state set to UNCONTROLLED')
        except Exception as e:
            self.get_logger().warning(f"Failed to initialize AVC A motor: {e}")
            self.avc_a_state = AVCState.ERROR
            self.avc_a_Solo = None
            self.avc_a_homed = False
            self.get_logger().info('AVC A state set to ERROR')
        
        # AVC B Mechanism
        try:
            self.avc_b_Solo = self.motor_init('avc_b')
            self.avc_b_state = AVCState.UNCONTROLLED
            self.avc_b_homed = False
            self.get_logger().info('AVC B state set to UNCONTROLLED')
        except Exception as e:
            self.get_logger().warning(f"Failed to initialize AVC B motor: {e}")
            self.avc_b_state = AVCState.ERROR
            self.avc_b_Solo = None
            self.avc_b_homed = False
            self.get_logger().info('AVC B state set to ERROR')
    
        # Set up a function that constantly monitors the state machine
        self.state_machine_frequency = self.get_parameter('state_machine_params.frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / self.state_machine_frequency, self.GRASP_state_machine)

        # Set up a function that publishes data at a set frequency
        self.feedback_frequency = self.get_parameter('feedback_params.frequency').get_parameter_value().double_value
        self.feedback_timer = self.create_timer(1.0 / self.feedback_frequency, self.publish_data)

        # Add GRASP state subscribers
        self.subscription           = self.create_subscription(String, 'GRASP_flags', self.GRASP_external_flags, 10) #QoS arbitrarily set at 10

        
        # Add GRASP publishers
        if self.grapple_state != GrappleState.ERROR:
            self.pub_gra_motor_feedback = self.create_publisher(String,  'gra_motor_feedback', 10)
            self.pub_gra_motor_curr_iq  = self.create_publisher(Float64, 'gra_motor_current_iq', 10)
            self.pub_gra_motor_pos      = self.create_publisher(Int32,   'gra_motor_pos', 10)
            self.pub_gra_motor_vel      = self.create_publisher(Int32,   'gra_motor_vel', 10)
        if self.avc_a_state != AVCState.ERROR:
            self.pub_avc_a_motor_feedback = self.create_publisher(String,  'avc_a_motor_feedback', 10)
            self.pub_avc_a_motor_curr_iq  = self.create_publisher(Float64, 'avc_a_motor_current_iq', 10)
            self.pub_avc_a_motor_pos      = self.create_publisher(Int32,   'avc_a_motor_pos', 10)
            self.pub_avc_a_motor_vel      = self.create_publisher(Int32,   'avc_a_motor_vel', 10)
        if self.avc_b_state != AVCState.ERROR:
            self.pub_avc_b_motor_feedback = self.create_publisher(String,  'avc_b_motor_feedback', 10)
            self.pub_avc_b_motor_curr_iq  = self.create_publisher(Float64, 'avc_b_motor_current_iq', 10)
            self.pub_avc_b_motor_pos      = self.create_publisher(Int32,   'avc_b_motor_pos', 10)
            self.pub_avc_b_motor_vel      = self.create_publisher(Int32,   'avc_b_motor_vel', 10)
        
        self.get_logger().info('GRASP_node initiated with a periodic state machine.') 

    # Function to find available serial ports
    def find_available_ports(self):
        """
        Search for available serial devices and return a list of active ports.
        Returns:
            list: List of available serial port device paths.
        """
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
        """
        Initialize a Solo motor controller for the given motor name using parameters.
        Args:
            name (str): Name of the motor ('grapple', 'avc_a' or 'avc_b').
        Returns:
            SoloMotorControllerUart: Initialized Solo motor controller object.
        Raises:
            Exception: If the motor driver cannot be found on any available port.
        """
        # Get parameters for initialising motor 
        self.get_logger().info(f'Initializing {name} motor driver')
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
        for port in self.available_ports:
            self.get_logger().info(f"Attempting to connect to {name} motor driver over {port}")
            Solo = solo.SoloMotorControllerUart(port=port, baudrate=soloBaudrate, address=address, loggerLevel=logger_level)
            read_address = Solo.get_device_address()[0]
            self.get_logger().info(f"{port} reads back motor address as {read_address}")
            if read_address == address:
                self.get_logger().info(f"Successfully connected to {name} motor driver over {port}")
                self.available_ports.remove(port)
                connection_successful = True
                break
            else:
                self.get_logger().info(f"Could not find {name} motor driver over {port}")
                self.get_logger().info(f"Disconnecting from {port}")
                connection_successful = False
                Solo.serial_close()

        # If motor driver cannot be found throw exception
        if not connection_successful:
            raise Exception(f"Could not find {name} motor driver over any available port: {self.available_ports}")
        
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
        """
        Set the motor to position mode and command it to the specified position.
        Args:
            Solo: Solo motor controller object.
            position_reference (float): Target position value.
        """
        # This a function to request a specific position to the motor.
        Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
        self.get_logger().debug(f"Changing motor to position mode control and setting position reference {position_reference}")
        Solo.set_position_reference(position_reference)
        return

    def motor_speed_control(self, Solo, speed_reference):
        """
        Set the motor to speed mode and command it to the specified speed.
        Args:
            Solo: Solo motor controller object.
            speed_reference (float): Target speed value.
        """
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
        """
        Set the motor to torque mode and command it to the specified torque.
        Args:
            Solo: Solo motor controller object.
            torque_reference (float): Target torque value.
        """
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
        """
        Publish feedback data for Grapple and AVC motors to their respective ROS topics.
        """
        # This function publish the data we want to record for external analysis
        
        # ----------------- Grapple data ----------------------------------
        if self.grapple_state != GrappleState.ERROR:
            msg = String()
            msg.data = f"State: {self.grapple_state.name}"
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
        if self.avc_a_state != AVCState.ERROR:
            avc_msg = String()
            avc_msg.data = f"State: {self.avc_a_state.name}"
            self.pub_avc_a_motor_feedback.publish(avc_msg)

            #Position Feedback
            avc_motor_pos_msg = Int32()
            avc_motor_pos_msg.data = self.avc_a_motor_pos
            self.pub_avc_a_motor_pos.publish(avc_motor_pos_msg)
            
            #Velocity Feedback
            avc_motor_vel_msg = Int32()
            avc_motor_vel_msg.data = self.avc_a_motor_speed
            self.pub_avc_a_motor_vel.publish(avc_motor_vel_msg)
            
            #Current Phase Iq Feedback
            avc_current_iq_msg = Float64()
            avc_current_iq_msg.data = self.avc_a_motor_current
            self.pub_avc_a_motor_curr_iq.publish(avc_current_iq_msg)
            
        if self.avc_b_state != AVCState.ERROR:
            avc_msg = String()
            avc_msg.data = f"State: {self.avc_b_state.name}"
            self.pub_avc_b_motor_feedback.publish(avc_msg)

            #Position Feedback
            avc_motor_pos_msg = Int32()
            avc_motor_pos_msg.data = self.avc_b_motor_pos
            self.pub_avc_b_motor_pos.publish(avc_motor_pos_msg)
            
            #Velocity Feedback
            avc_motor_vel_msg = Int32()
            avc_motor_vel_msg.data = self.avc_b_motor_speed
            self.pub_avc_b_motor_vel.publish(avc_motor_vel_msg)
            
            #Current Phase Iq Feedback
            avc_current_iq_msg = Float64()
            avc_current_iq_msg.data = self.avc_b_motor_current
            self.pub_avc_b_motor_curr_iq.publish(avc_current_iq_msg)
        
    # EXTERNAL FLAG MANAGEMENT  
    def GRASP_external_flags(self, msg):
        """
        Callback for external flag commands received via the /GRASP_flags topic.
        Handles state transitions and motor actions based on received flag.
        Args:
            msg (String): ROS message containing the command.
        """
        command = msg.data   
        match command:
            case 'RESET':
                self.GRASP_command = GRASPCommand.RESET
                self.get_logger().info('RESET command received')
                self.GRASP_mode = GRASPMode.UNCONTROLLED
                self.get_logger().info('GRASP mode set to UNCONTROLLED')
            case 'HOME':
                self.GRASP_command = GRASPCommand.HOME
                self.get_logger().info('HOME command received')
            case 'LAUNCH_LOCK':
                self.GRASP_command = GRASPCommand.LAUNCH_LOCK
                self.get_logger().info('LAUNCH_LOCK command received')
            case 'FREE_FLIGHT':
                self.GRASP_command = GRASPCommand.FREE_FLIGHT
                self.get_logger().info('FREE_FLIGHT command received')
            case 'READY':
                self.GRASP_command = GRASPCommand.READY
                self.get_logger().info('READY command received')
            case 'TRIGGER':
                self.GRASP_command = GRASPCommand.TRIGGER
                self.get_logger().info('TRIGGER command received')
            case 'CLEARANCE':
                self.GRASP_command = GRASPCommand.CLEARANCE
                self.get_logger().info('CLEARANCE command received')
            case 'AVC_A_POS1':
                self.GRASP_command = GRASPCommand.AVC_A_POS1
                self.get_logger().info('AVC_A_POS1 command received')
            case 'AVC_A_POS1p5':
                self.GRASP_command = GRASPCommand.AVC_A_POS1p5
                self.get_logger().info('AVC_A_POS1p5 command received')
            case 'AVC_A_POS2':
                self.GRASP_command = GRASPCommand.AVC_A_POS2
                self.get_logger().info('AVC_A_POS2 command received')
            case 'AVC_A_RETRACT':
                self.GRASP_command = GRASPCommand.AVC_A_RETRACT
                self.get_logger().info('AVC_A_RETRACT command received')
            case 'AVC_B_POS1':
                self.GRASP_command = GRASPCommand.AVC_B_POS1
                self.get_logger().info('AVC_B_POS1 command received')
            case 'AVC_B_POS1p5':
                self.GRASP_command = GRASPCommand.AVC_B_POS1p5
                self.get_logger().info('AVC_B_POS1p5 command received')
            case 'AVC_B_POS2':
                self.GRASP_command = GRASPCommand.AVC_B_POS2
                self.get_logger().info('AVC_B_POS2 command received')
            case 'AVC_B_RETRACT':
                self.GRASP_command = GRASPCommand.AVC_B_RETRACT
                self.get_logger().info('AVC_B_RETRACT command received')
            case 'RELEASE':
                self.GRASP_command = GRASPCommand.RELEASE
                self.get_logger().info('RELEASE command received')
                
            case _: # Catch invalid command 
                self.get_logger().error('Unknown GRASP command received')

    # STATE MACHINE CODE
    def GRASP_state_machine(self):
        """
        Periodic state machine handler for Grapple and AVC mechanisms.
        Updates motor states and executes state-dependent actions.
        """

        # Get the current state from Solo controllers
        # Grapple mechanism state
        if self.grapple_state != GrappleState.ERROR:
            self.gra_motor_pos_ref, error = self.grapple_Solo.get_position_reference()
            self.gra_motor_speed_ref, error = self.grapple_Solo.get_speed_reference()
            self.gra_motor_torque_ref, error = self.grapple_Solo.get_torque_reference_iq()
            self.gra_motor_control_mode, error = self.grapple_Solo.get_control_mode()
            self.gra_motor_pos, error     = self.grapple_Solo.get_position_counts_feedback()
            self.gra_motor_speed, error   = self.grapple_Solo.get_speed_feedback()
            self.gra_motor_current, error = self.grapple_Solo.get_quadrature_current_iq_feedback()

        # AVC A mechanism state
        if self.avc_a_state != AVCState.ERROR:
            self.avc_a_motor_pos_ref, error = self.avc_a_Solo.get_position_reference()
            self.avc_a_motor_speed_ref, error = self.avc_a_Solo.get_speed_reference()
            self.avc_a_motor_torque_ref, error = self.avc_a_Solo.get_torque_reference_iq()
            self.avc_a_motor_control_mode, error = self.avc_a_Solo.get_control_mode()
            self.avc_a_motor_pos, error     = self.avc_a_Solo.get_position_counts_feedback()
            self.avc_a_motor_speed, error   = self.avc_a_Solo.get_speed_feedback()
            self.avc_a_motor_current, error = self.avc_a_Solo.get_quadrature_current_iq_feedback()

        # AVC B mechanism state
        if self.avc_b_state != AVCState.ERROR:
            self.avc_b_motor_pos_ref, error = self.avc_b_Solo.get_position_reference()
            self.avc_b_motor_speed_ref, error = self.avc_b_Solo.get_speed_reference()
            self.avc_b_motor_torque_ref, error = self.avc_b_Solo.get_torque_reference_iq()
            self.avc_b_motor_control_mode, error = self.avc_b_Solo.get_control_mode()
            self.avc_b_motor_pos, error     = self.avc_b_Solo.get_position_counts_feedback()
            self.avc_b_motor_speed, error   = self.avc_b_Solo.get_speed_feedback()
            self.avc_b_motor_current, error = self.avc_b_Solo.get_quadrature_current_iq_feedback()
            
        # Checking GRASP modes and calling relevant code
        match self.GRASP_mode:
            case GRASPMode.ERROR:
                """
                ERROR mode. GRASP does nothing.
                """
                pass
            case GRASPMode.UNCONTROLLED:
                """
                UNCONTROLLED mode. GRASP does nothing.
                Entry: RESET command received or startup
                Exit: HOME command received -> Transitions to AVC_A_HOME mode
                """
                # Entry actions:
                
                # Exit conditions
                if self.GRASP_command == GRASPCommand.HOME:
                    self.GRASP_mode = GRASPMode.AVC_A_HOME
                    self.get_logger().info('Changed GRASP mode to AVC_A_HOME.')

            case GRASPMode.LAUNCH_LOCKED:
                """
                LAUNCH_LOCKED mode. Not currently implemented.
                Entry: move AVC A mechanism to LAUNCH LOCK state, move AVC B mechanism to LAUNCH LOCK state
                STOP
                """
                pass
            
            case GRASPMode.GRAPPLE_HOME:
                """
                GRAPPLE_HOME mode.
                Entry: move grapple mechanism to HOME state
                Exit: 
                1) LAUNCH LOCK command received -> Transitions to LAUNCH_LOCKED mode
                2) FREE FLIGHT command received -> Transitions to FREE_FLIGHT mode
                """
                
                # Entry actions:
                if self.grapple_state != GrappleState.ERROR:
                    if self.grapple_state != GrappleState.HOME:
                        target_iq = -0.7 # [A] Configuration parameter to be added to config file at later date.
                        if self.gra_motor_torque_ref != target_iq or self.gra_motor_control_mode != solo.ControlMode.TORQUE_MODE:
                            self.motor_torque_control(self.grapple_Solo, target_iq)
                    
                        # Active actions:
                        if abs(self.gra_motor_current) >= abs(target_iq) and self.gra_motor_speed == 0:
                            self.get_logger().debug('Current target for grapple homing reached.')
                            self.motor_torque_control(self.grapple_Solo, 0)
                            self.get_logger().debug('Setting current position to 0 to complete HOMING')
                            # Resetting position to 0 QP. Setting state to HOME.
                            self.grapple_Solo.reset_position_to_zero()
                            self.grapple_Solo.set_position_reference(0)
                            # Changing grapple state to HOME
                            self.grapple_state = GrappleState.HOME
                            self.grapple_homed = True
                            self.get_logger().info('Changed grapple state to HOME.')
                        
                # Exit conditions
                if self.GRASP_command == GRASPCommand.LAUNCH_LOCK and (self.grapple_state == GrappleState.HOME or self.grapple_state == GrappleState.ERROR):
                    self.GRASP_mode = GRASPMode.LAUNCH_LOCKED
                    self.get_logger().info('Changed GRASP mode to LAUNCH_LOCKED.')
                elif self.GRASP_command == GRASPCommand.FREE_FLIGHT and (self.grapple_state == GrappleState.HOME or self.grapple_state == GrappleState.ERROR):
                    self.GRASP_mode = GRASPMode.FREE_FLIGHT
                    self.get_logger().info('Changed GRASP mode to FREE_FLIGHT.')
            
            case GRASPMode.AVC_A_HOME:
                """
                AVC_A_HOME mode.
                Entry: move AVC A mechanism to HOME state
                Exit: 
                AVC A mechanism reaches HOME state -> Transitions to AVC_B_HOME mode
                """
                
                # Entry actions:
                if self.avc_a_state != AVCState.ERROR:
                    if self.avc_a_state != AVCState.HOME:
                        target_iq = 0.08 # [A] Configuration parameter to be added to config file at later date.
                        if self.avc_a_motor_torque_ref != target_iq or self.avc_a_motor_control_mode != solo.ControlMode.TORQUE_MODE:
                            self.motor_torque_control(self.avc_a_Solo, target_iq)
                        # Active actions:
                        if abs(self.avc_a_motor_current) >= abs(target_iq) and self.avc_a_motor_speed == 0:
                            self.get_logger().debug('Current target for AVC A homing reached.')
                            self.motor_torque_control(self.avc_a_Solo, 0)
                            self.get_logger().debug('Setting current position to 0 to complete HOMING')
                            # Resetting position to 0 QP. Setting state to HOME.
                            self.avc_a_Solo.reset_position_to_zero()
                            self.avc_a_Solo.set_position_reference(0)
                            # Changing AVC A state to HOME
                            self.avc_a_state = AVCState.HOME
                            self.get_logger().info('Changed AVC A state to HOME.')
                            
                # Exit conditions
                if self.avc_a_state == AVCState.HOME or self.avc_a_state == AVCState.ERROR:
                    self.GRASP_mode = GRASPMode.AVC_B_HOME
                    self.get_logger().info('Changed GRASP mode to AVC_B_HOME.')
                
            case GRASPMode.AVC_B_HOME:
                """
                AVC_B_HOME mode.
                Entry: move AVC B mechanism to HOME state
                Exit: 
                AVC B mechanism reaches HOME state -> Transitions to GRAPPLE_HOME mode
                """
                
                # Entry actions:
                if self.avc_b_state != AVCState.ERROR:
                    if self.avc_b_state != AVCState.HOME:
                        target_iq = 0.08 # [A] Configuration parameter to be added to config file at later date.
                        if self.avc_b_motor_torque_ref != target_iq or self.avc_b_motor_control_mode != solo.ControlMode.TORQUE_MODE:
                            self.motor_torque_control(self.avc_b_Solo, target_iq)
                            
                        # Active actions:
                        if abs(self.avc_b_motor_current) >= abs(target_iq) and self.avc_b_motor_speed == 0:
                            self.get_logger().debug('Current target for AVC B homing reached.')
                            self.motor_torque_control(self.avc_b_Solo, 0)
                            self.get_logger().debug('Setting current position to 0 to complete HOMING')
                            # Resetting position to 0 QP. Setting state to HOME.
                            self.avc_b_Solo.reset_position_to_zero()
                            self.avc_b_Solo.set_position_reference(0)
                            # Changing AVC B state to HOME
                            self.avc_b_state = AVCState.HOME
                            self.get_logger().info('Changed AVC B state to HOME.')
                        
                # Exit conditions
                if self.avc_b_state == AVCState.HOME or self.avc_b_state == AVCState.ERROR:
                    self.GRASP_mode = GRASPMode.GRAPPLE_HOME
                    self.get_logger().info('Changed GRASP mode to GRAPPLE_HOME.')

            case GRASPMode.FREE_FLIGHT:
                """
                FREE_FLIGHT mode. Not currently implemented.
                Entry: move GRAPPLE mechanism to FREE_FLIGHT state
                Exit:
                READY command recieved -> Transitions to READY mode
                """
                # Entry actions:
                if self.grapple_state != GrappleState.ERROR:
                    if self.grapple_state != GrappleState.FREE_FLIGHT:
                        self.grapple_state = GrappleState.FREE_FLIGHT
                        self.get_logger().info('Changed grapple state to FREE_FLIGHT.')
                
                # Exit conditions
                if self.GRASP_command == GRASPCommand.READY:
                    self.GRASP_mode = GRASPMode.READY
                    self.get_logger().info('Changed GRASP mode to READY.')
                    
            case GRASPMode.READY:
                """
                READY mode.
                Entry: move GRAPPLE mechanism to OPEN state
                Exit: 
                TRIGGER command received -> Transitions to SOFT_DOCK mode
                """
                
                # Entry actions:
                if self.grapple_state != GrappleState.ERROR:
                    target_pos = 3900 # [QP] Configuration parameter to be added to config file at later date.
                    if self.gra_motor_pos_ref != target_pos or self.gra_motor_control_mode != solo.ControlMode.POSITION_MODE:
                        self.motor_position_control(self.grapple_Solo, target_pos)
                    # Active actions:
                    if self.gra_motor_pos >= target_pos:
                        self.get_logger().debug('Position target for grapple opening reached.')
                        self.motor_torque_control(self.grapple_Solo, 0)
                        self.grapple_state = GrappleState.OPEN
                        self.get_logger().info('Changed grapple state to OPEN.')
                
                # Exit conditions
                if self.GRASP_command == GRASPCommand.TRIGGER:
                    self.GRASP_mode = GRASPMode.SOFT_DOCK
                    self.get_logger().info('Changed GRASP mode to SOFT_DOCK.')
            
            case GRASPMode.SOFT_DOCK:
                """
                SOFT_DOCK mode.
                Entry: move GRAPPLE mechanism to SOFT_DOCK state
                Exit: 
                1) GRAPPLE mechanism reaches SOFT_DOCK state -> Transitions to HARD_DOCK mode
                2) CLEARANCE command received -> Transitions to CLEARANCE mode
                """
                # Entry actions:
                if self.grapple_state != GrappleState.ERROR:
                    target_speed = -2000 # [QPS] Configuration parameter to be added to config file at later date.
                    if self.gra_motor_speed_ref != target_speed or self.gra_motor_control_mode != solo.ControlMode.SPEED_MODE:
                        self.motor_speed_control(self.grapple_Solo, target_speed)
                # Active actions:
                    
                # Exit conditions:
                if self.GRASP_command == GRASPCommand.CLEARANCE:
                    self.GRASP_mode = GRASPMode.CLEARANCE
                    self.get_logger().info('Changed GRASP mode to CLEARANCE.')
                elif self.gra_motor_pos <= 3173: # [QP] Configuration parameter to be added to config file at later date. Need to find what this value is from Rhys and Palash
                    self.get_logger().debug('Position target for grapple soft docking reached.')
                    self.grapple_state = GrappleState.SOFT_DOCK
                    self.get_logger().info('Changed grapple state to SOFT_DOCK.')
                    self.GRASP_mode = GRASPMode.HARD_DOCK
                    self.get_logger().info('Changed GRASP mode to HARD_DOCK.')
                
            case GRASPMode.HARD_DOCK:
                """
                HARD_DOCK mode.
                Entry: move GRAPPLE mechanism to HARD_DOCK state
                Exit:
                1) CLEARANCE command received -> Transitions to CLEARANCE mode
                2) RELEASE command received -> Transitions to RELEASE mode
                3) AVC_A_POS1 command received -> Transitions to AVC_A_POS1 mode
                4) AVC_B_POS1 command received -> Transitions to AVC_B_POS1 mode
                """
                # Entry actions:
                if self.grapple_state != GrappleState.ERROR:
                    if self.grapple_state != GrappleState.HARD_DOCK:
                        if self.gra_motor_pos <= 3173: # [QP] Configuration parameter to be added to config file at later date.
                            target_iq = -0.7 # [A] Configuration parameter to be added to config file at later date.
                            if self.gra_motor_torque_ref != target_iq or self.gra_motor_control_mode != solo.ControlMode.TORQUE_MODE:
                                self.motor_torque_control(self.grapple_Solo, target_iq)
                            # Active actions:
                            if abs(self.gra_motor_current) >= abs(target_iq) and self.gra_motor_speed == 0:
                                self.get_logger().debug('Current target for grapple hard docking reached.')
                                self.grapple_state = GrappleState.HARD_DOCK
                                self.get_logger().info('Changed grapple state to HARD_DOCK.')
                                self.motor_torque_control(self.grapple_Solo, 0) # May need to apply torque to maintain force. To be tested.

                # Exit conditions
                if self.GRASP_command == GRASPCommand.CLEARANCE:
                    self.GRASP_mode = GRASPMode.CLEARANCE
                    self.get_logger().info('Changed GRASP mode to CLEARANCE.')
                elif self.GRASP_command == GRASPCommand.RELEASE:
                    self.GRASP_mode = GRASPMode.RELEASE
                    self.get_logger().info('Changed GRASP mode to RELEASE.')
                elif self.GRASP_command == GRASPCommand.AVC_A_POS1 and self.grapple_state == GrappleState.HARD_DOCK:
                    self.GRASP_mode = GRASPMode.AVC_A_POS1
                    self.get_logger().info('Changed GRASP mode to AVC_A_POS1.')
                elif self.GRASP_command == GRASPCommand.AVC_B_POS1 and self.grapple_state == GrappleState.HARD_DOCK:
                    self.GRASP_mode = GRASPMode.AVC_B_POS1
                    self.get_logger().info('Changed GRASP mode to AVC_B_POS1.')
                    
                    
            case GRASPMode.CLEARANCE:
                """
                CLEARANCE mode. (Not currently implemented)
                Entry: move GRAPPLE mechanism to CLEARANCE state
                STOP
                """
                pass
            
            case GRASPMode.AVC_A_POS1:
                """
                AVC_A_POS1 mode. (Not currently implemented)
                Entry: move AVC A mechanism to AVC_A_POS1 state
                Exit: 
                AVC_A_POS2 command received -> Transitions to AVC_A_POS2 mode
                """
                pass
            case GRASPMode.AVC_A_POS1p5:
                """
                AVC_A_POS1p5 mode. (Not currently implemented)
                Entry: move AVC A mechanism to AVC_A_POS1p5 state
                Exit:
                AVC_A_RETRACT command received -> Transitions to AVC_A_RETRACT mode
                """
                pass
            case GRASPMode.AVC_A_POS2:
                """
                AVC_A_POS2 mode. (Not currently implemented)
                Entry: move AVC A mechanism to AVC_A_POS2 state
                Exit:
                AVC_A_POS1p5 command received -> Transitions to AVC_A_POS1p5 mode
                """
                pass
            case GRASPMode.AVC_A_RETRACT:
                """
                AVC_A_RETRACT mode. (Not currently implemented)
                Entry: move AVC A mechanism to HOME state
                Exit:
                AVC A mechanism reaches HOME state -> Transitions to HARD_DOCK mode
                """
                pass
            case GRASPMode.AVC_B_POS1:
                """
                AVC_B_POS1 mode. (Not currently implemented)
                Entry: move AVC B mechanism to AVC_B_POS1 state
                Exit: 
                AVC_B_POS2 command received -> Transitions to AVC_B_POS2 mode
                """
                pass
            case GRASPMode.AVC_B_POS1p5:
                """
                AVC_B_POS1p5 mode. (Not currently implemented)
                Entry: move AVC B mechanism to AVC_B_POS1p5 state
                Exit:
                AVC_B_RETRACT command received -> Transitions to AVC_B_RETRACT mode
                """
                pass
            case GRASPMode.AVC_B_POS2:
                """
                AVC_B_POS2 mode. (Not currently implemented)
                Entry: move AVC B mechanism to AVC_B_POS2 state
                Exit:
                AVC_B_POS1p5 command received -> Transitions to AVC_B_POS1p5 mode
                """
                pass
            case GRASPMode.AVC_B_RETRACT:
                """
                AVC_B_RETRACT mode. (Not currently implemented)
                Entry: move AVC B mechanism to HOME state
                Exit:
                AVC B mechanism reaches HOME state -> Transitions to HARD_DOCK mode
                """
                pass
            case GRASPMode.RELEASE:
                """
                RELEASE mode.
                Entry: move GRAPPLE mechanism to CLEARANCE state
                Exit: 
                GRAPPLE mechanism reaches CLEARANCE state
                STOP
                """
                # Entry actions:
                if self.grapple_state != GrappleState.ERROR:
                    if self.grapple_state != GrappleState.CLEARANCE:
                        target_pos = 3600 # [QP] Configuration parameter to be added to config file
                        if self.gra_motor_pos_ref != target_pos or self.gra_motor_control_mode != solo.ControlMode.POSITION_MODE:
                            self.motor_position_control(self.grapple_Solo, target_pos)
                        # Active actions:
                        if self.gra_motor_pos >= target_pos:
                            self.get_logger().debug('Position target for grapple releasing reached.')
                            self.grapple_state = GrappleState.CLEARANCE
                            self.get_logger().info('Changed grapple state to CLEARANCE.')
                            self.motor_torque_control(self.grapple_Solo, 0)
                # Exit conditions
                if self.grapple_state == GrappleState.CLEARANCE:
                    pass
                            
            case GRASPMode.HEATER_ON:
                """
                HEATER_ON mode. Not currently implemented.
                """
                pass
            case GRASPMode.HEATER_OFF:
                """
                HEATER_OFF mode. Not currently implemented.
                """
                pass
            
            case _:
                # Catch invalid mode
                self.get_logger().error('Unknown GRASP mode encountered')
        
def main():
    """
    Main entry point for the GRASP_node ROS2 node.
    Initializes ROS, creates the node, and spins until shutdown.
    """
    rclpy.init()
    GRASP_node = GRASPNode()
    rclpy.spin(GRASP_node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()