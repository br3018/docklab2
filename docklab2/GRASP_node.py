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
from abc import ABC, abstractmethod
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
    NONE = auto()
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
    UNCONTROLLED = auto()
    LAUNCH_LOCK = auto()
    HOME = auto()
    POS1 = auto()
    POS1p5 = auto()
    POS2 = auto()

# Define base mechanism controller class
class MechanismController(ABC):
    """
    Base class for motor-driven mechanism controllers.
    
    Provides common functionality for:
    - Motor initialization and configuration
    - Control mode management (position, speed, torque)
    - Feedback reading (position, speed, current)
    - State management
    
    Subclasses must implement state-specific movement methods.
    """
    
    def __init__(self, name: str, motor_params: dict, logger, available_ports: list):
        """
        Initialize the mechanism controller.
        
        Args:
            name: Name of the mechanism (e.g., 'grapple', 'avc_a', 'avc_b')
            motor_params: Dictionary containing all motor configuration parameters
            logger: ROS logger instance for logging
            available_ports: List of available serial ports to search
        """
        self.name = name
        self.logger = logger
        self.motor_params = motor_params
        self._state = None
        self._solo = None
        
        # Motor feedback variables
        self.position = 0
        self.speed = 0
        self.current = 0
        self.position_ref = 0
        self.speed_ref = 0
        self.torque_ref = 0.0
        self.control_mode = None
        
        # Initialize the motor
        try:
            self._solo = self._initialize_motor(available_ports)
            self._state = self._get_initial_state()
            self.logger.info(f"{self.name} initialized successfully in {self._state.name} state")
        except Exception as e:
            self.logger.error(f"Failed to initialize {self.name} motor: {e}")
    
    @abstractmethod
    def _get_initial_state(self):
        """Return the initial state for this mechanism type."""
        pass
    
    def _initialize_motor(self, available_ports: list):
        """
        Initialize and configure the Solo motor controller.
        
        Args:
            available_ports: List of available serial ports
            
        Returns:
            SoloMotorControllerUart: Configured motor controller instance
            
        Raises:
            Exception: If motor cannot be found or initialized
        """
        params = self.motor_params
        
        # Parse baudrate
        if params['baudrate'] == 937500:
            solo_baudrate = solo.UartBaudRate.RATE_937500
        else:
            raise ValueError(f"Unsupported baudrate: {params['baudrate']}")
        
        # Parse enums
        command_mode_map = {'DIGITAL': solo.CommandMode.DIGITAL}
        motor_type_map = {'BLDC_PMSM': solo.MotorType.BLDC_PMSM}
        feedback_control_mode_map = {'HALL_SENSORS': solo.FeedbackControlMode.HALL_SENSORS}
        control_mode_map = {'POSITION_MODE': solo.ControlMode.POSITION_MODE}
        
        command_mode = command_mode_map.get(params['command_mode'])
        motor_type = motor_type_map.get(params['motor_type'])
        feedback_control_mode = feedback_control_mode_map.get(params['feedback_control_mode'])
        control_mode = control_mode_map.get(params['control_mode'])
        
        if not all([command_mode, motor_type, feedback_control_mode, control_mode]):
            raise ValueError(f"Invalid motor configuration parameters for {self.name}")
        
        # Connect to motor driver
        self.logger.info(f"Attempting to connect to {self.name} motor driver")
        solo_controller = None
        connected_port = None
        
        for port in available_ports:
            self.logger.debug(f"Trying port {port}")
            try:
                solo_controller = solo.SoloMotorControllerUart(
                    port=port,
                    baudrate=solo_baudrate,
                    address=params['address'],
                    loggerLevel=params['logger_level']
                )
                
                read_address = solo_controller.get_device_address()[0]
                self.logger.debug(f"{port} reads back address {read_address}")
                
                if read_address == params['address']:
                    connected_port = port
                    self.logger.info(f"Connected to {self.name} motor on {port}")
                    break
                else:
                    solo_controller.serial_close()
                    
            except Exception as e:
                self.logger.debug(f"Failed to connect on {port}: {e}")
                continue
        
        if not solo_controller or not connected_port:
            raise Exception(f"Could not find {self.name} motor on any available port")
        
        # Remove used port from available list
        available_ports.remove(connected_port)
        
        # Configure motor parameters
        self.logger.debug(f"Configuring {self.name} motor parameters")
        solo_controller.reset_position_to_zero()
        solo_controller.set_output_pwm_frequency_khz(params['pwm_frequency_khz'])
        solo_controller.set_current_limit(params['current_limit'])
        solo_controller.set_motor_poles_counts(params['motor_poles_counts'])
        solo_controller.set_command_mode(command_mode)
        solo_controller.set_motor_type(motor_type)
        solo_controller.set_feedback_control_mode(feedback_control_mode)
        solo_controller.set_current_controller_kp(params['current_controller_kp'])
        solo_controller.set_current_controller_ki(params['current_controller_ki'])
        solo_controller.set_speed_controller_kp(params['speed_controller_kp'])
        solo_controller.set_speed_controller_ki(params['speed_controller_ki'])
        solo_controller.set_position_controller_kp(params['position_controller_kp'])
        solo_controller.set_position_controller_ki(params['position_controller_ki'])
        solo_controller.set_control_mode(control_mode)
        solo_controller.set_speed_limit(params['speed_limit'])
        solo_controller.set_speed_acceleration_value(params['speed_acceleration'])
        solo_controller.set_speed_deceleration_value(params['speed_deceleration'])
        
        self.logger.info(f"{self.name} motor configured successfully")
        return solo_controller
    
    def update_feedback(self):
        """
        Read current feedback values from the motor controller.
        Updates position, speed, current, and reference values.
        """
        if not self.is_initialised():
            return
        
        try:
            self.position_ref, _ = self._solo.get_position_reference()
            self.speed_ref, _ = self._solo.get_speed_reference()
            self.torque_ref, _ = self._solo.get_torque_reference_iq()
            self.control_mode, _ = self._solo.get_control_mode()
            self.position, _ = self._solo.get_position_counts_feedback()
            self.speed, _ = self._solo.get_speed_feedback()
            self.current, _ = self._solo.get_quadrature_current_iq_feedback()
        except Exception as e:
            self.logger.error(f"Error reading feedback from {self.name}: {e}")
    
    def position_control(self, position_reference: float):
        """
        Command the motor to a specific position using position control mode.
        
        Args:
            position_reference: Target position in encoder counts
        """
        if not self.is_initialised():
            return
        
        self.logger.debug(f"{self.name}: Changing motor to position mode control and setting position reference {position_reference}")
        self._solo.set_control_mode(solo.ControlMode.POSITION_MODE)
        self._solo.set_position_reference(position_reference)
    
    def speed_control(self, speed_reference: float):
        """
        Command the motor to a specific speed using speed control mode.
        
        Args:
            speed_reference: Target speed in RPM (negative for clockwise)
        """
        if not self.is_initialised():
            return
        
        self._solo.set_control_mode(solo.ControlMode.SPEED_MODE)
        
        if speed_reference < 0:
            self._solo.set_motor_direction(solo.Direction.CLOCKWISE)
            self.logger.debug(f"{self.name}: Changing motor to speed mode control and setting speed reference {speed_reference} RPM (CW)")
        else:
            self._solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
            self.logger.debug(f"{self.name}: Changing motor to speed mode control and setting speed reference {speed_reference} RPM (CCW)")

        speed_reference = abs(speed_reference)
        self._solo.set_speed_reference(speed_reference)
    
    def torque_control(self, torque_reference: float):
        """
        Command the motor to a specific torque using torque control mode.
        
        Args:
            torque_reference: Target torque in Amperes (negative for clockwise)
        """
        if not self.is_initialised():
            return
        
        self._solo.set_control_mode(solo.ControlMode.TORQUE_MODE)
        
        if torque_reference < 0:
            self._solo.set_motor_direction(solo.Direction.CLOCKWISE)
            torque_reference = abs(torque_reference)
            self.logger.debug(f"{self.name}: Changing motor to torque mode control and setting torque reference {torque_reference} A (CW)")
        else:
            self._solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
            self.logger.debug(f"{self.name}: Changing motor to torque mode control and setting torque reference {torque_reference} A (CCW)")

        self._solo.set_torque_reference_iq(torque_reference)
    
    def reset_position(self):
        """Reset the motor position counter to zero."""
        if not self.is_initialised():
            return
        
        self._solo.reset_position_to_zero()
        self._solo.set_position_reference(0)
        self.logger.debug(f"{self.name}: Position reset to zero")
    
    def get_state(self):
        """Get the current state of the mechanism."""
        return self._state
    
    def is_initialised(self) -> bool:
        """Check if the mechanism is initialized. True if initialized, False otherwise."""
        return self._solo is not None
    
class GrappleController(MechanismController):
    """
    Controller for the grapple mechanism.

    Manages states: HOME, FREE_FLIGHT, OPEN, SOFT_DOCK, HARD_DOCK, CLEARANCE
    Provides methods for homing and moving between operational states.
    """
    
    # Configuration constants (should be loaded from config file in production)
    HOME_CURRENT_TARGET = -0.7  # [A]
    OPEN_POSITION_TARGET = 3900  # [QP]
    SOFT_DOCK_POSITION_THRESHOLD = 3173  # [QP]
    HARD_DOCK_CURRENT_TARGET = -0.7  # [A]
    CLEARANCE_POSITION_TARGET = 3600  # [QP]
    SOFT_DOCK_SPEED_TARGET = -2000  # [QPS]
    
    def __init__(self, motor_params: dict, logger, available_ports: list):
        """
        Initialize the grapple controller.
        
        Args:
            motor_params: Motor configuration parameters
            logger: ROS logger instance
            available_ports: List of available serial ports
        """
        super().__init__('grapple', motor_params, logger, available_ports)
    
    def _get_initial_state(self):
        """Return the initial state for grapple mechanism."""
        return GrappleState.UNCONTROLLED
    
    def home(self) -> bool:
        """
        Move the grapple mechanism to HOME position using current control.
        
        Returns:
            bool: True if homing is complete or motor not initialised, False if still in progress
        """
        if self._state == GrappleState.HOME:
            return True
        
        if not self.is_initialised():
            self.logger.warning(f"{self.name} not initialised, simulating HOME state")
            self._state = GrappleState.HOME
            return True
        
        # Apply homing current if not already set
        if self.torque_ref != self.HOME_CURRENT_TARGET or \
           self.control_mode != solo.ControlMode.TORQUE_MODE:
            self.torque_control(self.HOME_CURRENT_TARGET)
        
        # Check if homing is complete (current reached and motor stopped)
        if abs(self.current) >= abs(self.HOME_CURRENT_TARGET) and self.speed == 0:
            self.logger.debug(f"{self.name} homing current target reached")
            self.torque_control(0)
            self.reset_position()
            self._state = GrappleState.HOME
            self.logger.info(f"{self.name} reached HOME state")
            return True
        
        return False
    
    def free_flight(self):
        """
        Move the grapple mechanism to FREE_FLIGHT state.
        
        Note: Implementation not yet defined. Placeholder for future development.
        """
        if self._state == GrappleState.FREE_FLIGHT:
            return True
        
        # TODO: Implement FREE_FLIGHT movement logic
        self._state = GrappleState.FREE_FLIGHT
        self.logger.info(f"{self.name} moved to FREE_FLIGHT state")
        return False
    
    def open(self) -> bool:
        """
        Move the grapple mechanism to OPEN position.
        
        Returns:
            bool: True if opening is complete, False if still in progress
        """
        if self._state == GrappleState.OPEN:
            return True
        
        if not self.is_initialised():
            self.logger.warning(f"{self.name} not initialised, simulating OPEN state")
            self._state = GrappleState.OPEN
            return True
        
        # Command position if not already set
        if self.position_ref != self.OPEN_POSITION_TARGET or \
           self.control_mode != solo.ControlMode.POSITION_MODE:
            self.position_control(self.OPEN_POSITION_TARGET)
        
        # Check if target reached
        if self.position >= self.OPEN_POSITION_TARGET:
            self.logger.debug(f"{self.name} opening position target reached")
            self.torque_control(0)
            self._state = GrappleState.OPEN
            self.logger.info(f"{self.name} reached OPEN state")
            return True
        
        return False
    
    def soft_dock(self) -> bool:
        """
        Move the grapple mechanism to SOFT_DOCK position using speed control.
        
        Returns:
            bool: True if soft docking is complete, False if still in progress
        """
        if self._state == GrappleState.SOFT_DOCK:
            return True
        
        if not self.is_initialised():
            self.logger.warning(f"{self.name} not initialised, simulating SOFT_DOCK state")
            self._state = GrappleState.SOFT_DOCK
            return True
        
        # Apply speed control if not already set
        if self.speed_ref != self.SOFT_DOCK_SPEED_TARGET or \
           self.control_mode != solo.ControlMode.SPEED_MODE:
            self.speed_control(self.SOFT_DOCK_SPEED_TARGET)
        
        # Check if soft dock position reached
        if self.position <= self.SOFT_DOCK_POSITION_THRESHOLD:
            self.logger.debug(f"{self.name} soft dock position reached")
            self._state = GrappleState.SOFT_DOCK
            self.logger.info(f"{self.name} reached SOFT_DOCK state")
            return True
        
        return False
    
    def hard_dock(self) -> bool:
        """
        Move the grapple mechanism to HARD_DOCK position using current control.
        
        Returns:
            bool: True if hard docking is complete, False if still in progress
        """
        if self._state == GrappleState.HARD_DOCK:
            return True
        
        if not self.is_initialised():
            self.logger.warning(f"{self.name} not initialised, simulating HARD_DOCK state")
            self._state = GrappleState.HARD_DOCK
            return True
        
        # Apply hard docking current if not already set
        if self.torque_ref != self.HARD_DOCK_CURRENT_TARGET or \
           self.control_mode != solo.ControlMode.TORQUE_MODE:
            self.torque_control(self.HARD_DOCK_CURRENT_TARGET)
        
        # Check if hard docking is complete
        if abs(self.current) >= abs(self.HARD_DOCK_CURRENT_TARGET) and self.speed == 0:
            self.logger.debug(f"{self.name} hard dock current target reached")
            self.torque_control(0)  # May need to maintain force - to be tested
            self._state = GrappleState.HARD_DOCK
            self.logger.info(f"{self.name} reached HARD_DOCK state")
            return True
        
        return False
    
    def clearance(self) -> bool:
        """
        Move the grapple mechanism to CLEARANCE position (release).
        
        Returns:
            bool: True if clearance is complete, False if still in progress
        """
        if self._state == GrappleState.CLEARANCE:
            return True
        
        if not self.is_initialised():
            self.logger.warning(f"{self.name} not initialised, simulating CLEARANCE state")
            self._state = GrappleState.CLEARANCE
            return True
        
        # Command clearance position if not already set
        if self.position_ref != self.CLEARANCE_POSITION_TARGET or \
           self.control_mode != solo.ControlMode.POSITION_MODE:
            self.position_control(self.CLEARANCE_POSITION_TARGET)
        
        # Check if target reached
        if self.position >= self.CLEARANCE_POSITION_TARGET:
            self.logger.debug(f"{self.name} clearance position target reached")
            self.torque_control(0)
            self._state = GrappleState.CLEARANCE
            self.logger.info(f"{self.name} reached CLEARANCE state")
            return True
        
        return False

class AVCController(MechanismController):
    """
    Controller for AVC (Active Valve Core) mechanisms.
    
    Manages states: HOME, LAUNCH_LOCK, POS1, POS1p5, POS2
    Provides methods for homing and moving between positions.
    """
    
    # Configuration constants (should be loaded from config file in production)
    HOME_CURRENT_TARGET = 0.08  # [A]
    POS1_TARGET = 1000  # [QP] - Placeholder value, needs configuration
    POS1P5_TARGET = 1500  # [QP] - Placeholder value, needs configuration
    POS2_TARGET = 2000  # [QP] - Placeholder value, needs configuration
    
    def __init__(self, name: str, motor_params: dict, logger, available_ports: list):
        """
        Initialize the AVC controller.
        
        Args:
            name: Name of the AVC mechanism ('avc_a' or 'avc_b')
            motor_params: Motor configuration parameters
            logger: ROS logger instance
            available_ports: List of available serial ports
        """
        if name not in ['avc_a', 'avc_b']:
            raise ValueError(f"Invalid AVC name: {name}. Must be 'avc_a' or 'avc_b'")
        super().__init__(name, motor_params, logger, available_ports)
    
    def _get_initial_state(self):
        """Return the initial state for AVC mechanism."""
        return AVCState.UNCONTROLLED
    
    def home(self) -> bool:
        """
        Move the AVC mechanism to HOME position using current control.
        
        Returns:
            bool: True if homing is complete, False if still in progress
        """
        if self._state == AVCState.HOME:
            return True
        
        if not self.is_initialised():
            self.logger.warning(f"{self.name} not initialised, simulating HOME state")
            self._state = AVCState.HOME
            return True
        
        # Apply homing current if not already set
        if self.torque_ref != self.HOME_CURRENT_TARGET or \
           self.control_mode != solo.ControlMode.TORQUE_MODE:
            self.torque_control(self.HOME_CURRENT_TARGET)
        
        # Check if homing is complete
        if abs(self.current) >= abs(self.HOME_CURRENT_TARGET) and self.speed == 0:
            self.logger.debug(f"{self.name} homing current target reached")
            self.torque_control(0)
            self.reset_position()
            self._state = AVCState.HOME
            self.logger.info(f"{self.name} reached HOME state")
            return True
        
        return False
    
    def launch_lock(self):
        """
        Move the AVC mechanism to LAUNCH_LOCK state.
        
        Note: Implementation not yet defined. Placeholder for future development.
        """
        if self._state == AVCState.LAUNCH_LOCK:
            return True
        
        # TODO: Implement LAUNCH_LOCK movement logic
        self._state = AVCState.LAUNCH_LOCK
        self.logger.info(f"{self.name} moved to LAUNCH_LOCK state")
        return False
    
    def pos1(self) -> bool:
        """
        Move the AVC mechanism to POS1.
        
        Returns:
            bool: True if position is reached, False if still in progress
        """
        if self._state == AVCState.POS1:
            return True
        
        if not self.is_initialised():
            self.logger.warning(f"{self.name} not initialised, simulating POS1 state")
            self._state = AVCState.POS1
            return True
        
        # Command position if not already set
        if self.position_ref != self.POS1_TARGET or \
           self.control_mode != solo.ControlMode.POSITION_MODE:
            self.position_control(self.POS1_TARGET)
        
        # Check if target reached (with small tolerance)
        if abs(self.position - self.POS1_TARGET) <= 10:
            self.logger.debug(f"{self.name} POS1 target reached")
            self._state = AVCState.POS1
            self.logger.info(f"{self.name} reached POS1 state")
            return True
        
        return False
    
    def pos1p5(self) -> bool:
        """
        Move the AVC mechanism to POS1.5 (intermediate position).
        
        Returns:
            bool: True if position is reached, False if still in progress
        """
        if self._state == AVCState.POS1p5:
            return True
        
        if not self.is_initialised():
            self.logger.warning(f"{self.name} not initialised, simulating POS1.5 state")
            self._state = AVCState.POS1p5
            return True 
        
        # Command position if not already set
        if self.position_ref != self.POS1P5_TARGET or \
           self.control_mode != solo.ControlMode.POSITION_MODE:
            self.position_control(self.POS1P5_TARGET)
        
        # Check if target reached
        if abs(self.position - self.POS1P5_TARGET) <= 10:
            self.logger.debug(f"{self.name} POS1.5 target reached")
            self._state = AVCState.POS1p5
            self.logger.info(f"{self.name} reached POS1.5 state")
            return True
        
        return False
    
    def pos2(self) -> bool:
        """
        Move the AVC mechanism to POS2.
        
        Returns:
            bool: True if position is reached, False if still in progress
        """
        if self._state == AVCState.POS2:
            return True
        
        if not self.is_initialised():
            self.logger.warning(f"{self.name} not initialised, simulating POS2 state")
            self._state = AVCState.POS2
            return True
        
        # Command position if not already set
        if self.position_ref != self.POS2_TARGET or \
           self.control_mode != solo.ControlMode.POSITION_MODE:
            self.position_control(self.POS2_TARGET)
        
        # Check if target reached
        if abs(self.position - self.POS2_TARGET) <= 10:
            self.logger.debug(f"{self.name} POS2 target reached")
            self._state = AVCState.POS2
            self.logger.info(f"{self.name} reached POS2 state")
            return True
        
        return False

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
        
        # Initialize motor controllers
        self.get_logger().debug('Initializing motor controllers')
        
        # Helper function to extract motor parameters
        def get_motor_params(motor_name: str) -> dict:
            """Extract motor parameters from ROS parameters for a specific motor."""
            idx = self.get_parameter('solo_params.name').get_parameter_value().string_array_value.index(motor_name)
            return {
                'address': self.get_parameter('solo_params.address').get_parameter_value().integer_array_value[idx],
                'baudrate': self.get_parameter('solo_params.baudrate').get_parameter_value().integer_array_value[idx],
                'pwm_frequency_khz': self.get_parameter('solo_params.pwm_frequency_khz').get_parameter_value().integer_array_value[idx],
                'current_limit': self.get_parameter('solo_params.current_limit').get_parameter_value().integer_array_value[idx],
                'motor_poles_counts': self.get_parameter('solo_params.motor_poles_counts').get_parameter_value().integer_array_value[idx],
                'command_mode': self.get_parameter('solo_params.command_mode').get_parameter_value().string_array_value[idx],
                'motor_type': self.get_parameter('solo_params.motor_type').get_parameter_value().string_array_value[idx],
                'feedback_control_mode': self.get_parameter('solo_params.feedback_control_mode').get_parameter_value().string_array_value[idx],
                'current_controller_kp': self.get_parameter('solo_params.current_controller_kp').get_parameter_value().double_array_value[idx],
                'current_controller_ki': self.get_parameter('solo_params.current_controller_ki').get_parameter_value().double_array_value[idx],
                'speed_controller_kp': self.get_parameter('solo_params.speed_controller_kp').get_parameter_value().double_array_value[idx],
                'speed_controller_ki': self.get_parameter('solo_params.speed_controller_ki').get_parameter_value().double_array_value[idx],
                'position_controller_kp': self.get_parameter('solo_params.position_controller_kp').get_parameter_value().double_array_value[idx],
                'position_controller_ki': self.get_parameter('solo_params.position_controller_ki').get_parameter_value().double_array_value[idx],
                'control_mode': self.get_parameter('solo_params.control_mode').get_parameter_value().string_array_value[idx],
                'speed_limit': self.get_parameter('solo_params.speed_limit').get_parameter_value().integer_array_value[idx],
                'speed_acceleration': self.get_parameter('solo_params.speed_acceleration').get_parameter_value().integer_array_value[idx],
                'speed_deceleration': self.get_parameter('solo_params.speed_deceleration').get_parameter_value().integer_array_value[idx],
                'logger_level': self.get_parameter('solo_params.logger_level').get_parameter_value().integer_array_value[idx],
            }
        
        # Initialize Grapple Controller
        grapple_params = get_motor_params('grapple')
        self.grapple_controller = GrappleController(grapple_params, self.get_logger(), self.available_ports)

        # Initialize AVC A Controller
        avc_a_params = get_motor_params('avc_a')
        self.avc_a_controller = AVCController('avc_a', avc_a_params, self.get_logger(), self.available_ports)
        
        # Initialize AVC B Controller
        avc_b_params = get_motor_params('avc_b')
        self.avc_b_controller = AVCController('avc_b', avc_b_params, self.get_logger(), self.available_ports)
    
        # Set up a function that constantly monitors the state machine
        self.state_machine_frequency = self.get_parameter('state_machine_params.frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / self.state_machine_frequency, self.GRASP_state_machine)

        # Set up a function that publishes data at a set frequency
        self.feedback_frequency = self.get_parameter('feedback_params.frequency').get_parameter_value().double_value
        self.feedback_timer = self.create_timer(1.0 / self.feedback_frequency, self.publish_data)

        # Add GRASP state subscribers
        self.subscription           = self.create_subscription(String, 'GRASP_flags', self.GRASP_external_flags, 10) #QoS arbitrarily set at 10

        
        # Add GRASP publishers
        if self.grapple_controller.is_initialised():
            self.pub_gra_motor_feedback = self.create_publisher(String,  'gra_motor_feedback', 10)
            self.pub_gra_motor_curr_iq  = self.create_publisher(Float64, 'gra_motor_current_iq', 10)
            self.pub_gra_motor_pos      = self.create_publisher(Int32,   'gra_motor_pos', 10)
            self.pub_gra_motor_vel      = self.create_publisher(Int32,   'gra_motor_vel', 10)
        if self.avc_a_controller.is_initialised():
            self.pub_avc_a_motor_feedback = self.create_publisher(String,  'avc_a_motor_feedback', 10)
            self.pub_avc_a_motor_curr_iq  = self.create_publisher(Float64, 'avc_a_motor_current_iq', 10)
            self.pub_avc_a_motor_pos      = self.create_publisher(Int32,   'avc_a_motor_pos', 10)
            self.pub_avc_a_motor_vel      = self.create_publisher(Int32,   'avc_a_motor_vel', 10)
        if self.avc_b_controller.is_initialised():
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

    # Function to publish data into topics
    def publish_data(self):
        """
        Publish feedback data for Grapple and AVC motors to their respective ROS topics.
        """
        # This function publish the data we want to record for external analysis
        
        # ----------------- Grapple data ----------------------------------
        if self.grapple_controller.is_initialised():
            msg = String()
            msg.data = f"State: {self.grapple_controller.get_state().name}"
            self.pub_gra_motor_feedback.publish(msg)

            #Position Feedback
            grapple_motor_pos_msg = Int32()
            grapple_motor_pos_msg.data = self.grapple_controller.position
            self.pub_gra_motor_pos.publish(grapple_motor_pos_msg)
            
            #Velocity Feedback
            grapple_motor_vel_msg = Int32()
            grapple_motor_vel_msg.data = self.grapple_controller.speed
            self.pub_gra_motor_vel.publish(grapple_motor_vel_msg)
            
            #Current Phase Iq Feedback
            grapple_current_iq_msg = Float64()
            grapple_current_iq_msg.data = self.grapple_controller.current
            self.pub_gra_motor_curr_iq.publish(grapple_current_iq_msg)
        
        # ----------------- AVC A data -----------------------------------
        if self.avc_a_controller.is_initialised():
            avc_msg = String()
            avc_msg.data = f"State: {self.avc_a_controller.get_state().name}"
            self.pub_avc_a_motor_feedback.publish(avc_msg)

            #Position Feedback
            avc_motor_pos_msg = Int32()
            avc_motor_pos_msg.data = self.avc_a_controller.position
            self.pub_avc_a_motor_pos.publish(avc_motor_pos_msg)
            
            #Velocity Feedback
            avc_motor_vel_msg = Int32()
            avc_motor_vel_msg.data = self.avc_a_controller.speed
            self.pub_avc_a_motor_vel.publish(avc_motor_vel_msg)
            
            #Current Phase Iq Feedback
            avc_current_iq_msg = Float64()
            avc_current_iq_msg.data = self.avc_a_controller.current
            self.pub_avc_a_motor_curr_iq.publish(avc_current_iq_msg)
            
        # ----------------- AVC B data -----------------------------------
        if self.avc_b_controller.is_initialised():
            avc_msg = String()
            avc_msg.data = f"State: {self.avc_b_controller.get_state().name}"
            self.pub_avc_b_motor_feedback.publish(avc_msg)

            #Position Feedback
            avc_motor_pos_msg = Int32()
            avc_motor_pos_msg.data = self.avc_b_controller.position
            self.pub_avc_b_motor_pos.publish(avc_motor_pos_msg)
            
            #Velocity Feedback
            avc_motor_vel_msg = Int32()
            avc_motor_vel_msg.data = self.avc_b_controller.speed
            self.pub_avc_b_motor_vel.publish(avc_motor_vel_msg)
            
            #Current Phase Iq Feedback
            avc_current_iq_msg = Float64()
            avc_current_iq_msg.data = self.avc_b_controller.current
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

        # Update feedback from all controllers
        self.grapple_controller.update_feedback()
        self.avc_a_controller.update_feedback()
        self.avc_b_controller.update_feedback()
            
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
                # --- Ongoing Actions ---
                # No actions in uncontrolled state
                
                # --- Command-triggered Transitions ---
                if self.GRASP_command == GRASPCommand.HOME:
                    self.GRASP_mode = GRASPMode.AVC_A_HOME
                    self.GRASP_command = GRASPCommand.NONE
                    self.get_logger().info('Changed GRASP mode to AVC_A_HOME.')
                    return
            
            case GRASPMode.AVC_A_HOME:
                """
                AVC_A_HOME mode.
                Entry: move AVC A mechanism to HOME state
                Exit: 
                AVC A mechanism reaches HOME state -> Transitions to AVC_B_HOME mode
                """
                
                # --- Entry/Ongoing Actions ---
                action_complete = self.avc_a_controller.home()
                
                # --- Completion-based Transitions ---
                if action_complete:
                    self.GRASP_mode = GRASPMode.AVC_B_HOME
                    self.get_logger().info('Changed GRASP mode to AVC_B_HOME.')
                    return
                
            case GRASPMode.AVC_B_HOME:
                """
                AVC_B_HOME mode.
                Entry: move AVC B mechanism to HOME state
                Exit: 
                AVC B mechanism reaches HOME state -> Transitions to GRAPPLE_HOME mode
                """
                
                # --- Entry/Ongoing Actions ---
                action_complete = self.avc_b_controller.home()
                
                # --- Completion-based Transitions ---
                if action_complete:
                    self.GRASP_mode = GRASPMode.GRAPPLE_HOME
                    self.get_logger().info('Changed GRASP mode to GRAPPLE_HOME.')
                    return
            
            case GRASPMode.GRAPPLE_HOME:
                """
                GRAPPLE_HOME mode.
                Entry: move grapple mechanism to HOME state
                Exit: 
                1) LAUNCH LOCK command received -> Transitions to LAUNCH_LOCKED mode
                2) FREE FLIGHT command received -> Transitions to FREE_FLIGHT mode
                """
                
                # --- Entry/Ongoing Actions ---
                action_complete = self.grapple_controller.home()
                
                # --- Completion-based Transitions ---
                if not action_complete:
                    return
                        
                # --- Command-triggered Transitions ---
                if self.GRASP_command == GRASPCommand.LAUNCH_LOCK:
                    self.GRASP_mode = GRASPMode.LAUNCH_LOCKED
                    self.GRASP_command = GRASPCommand.NONE
                    self.get_logger().info('Changed GRASP mode to LAUNCH_LOCKED.')
                    return
                    
                if self.GRASP_command == GRASPCommand.FREE_FLIGHT:
                    self.GRASP_mode = GRASPMode.FREE_FLIGHT
                    self.GRASP_command = GRASPCommand.NONE
                    self.get_logger().info('Changed GRASP mode to FREE_FLIGHT.')
                    return

            case GRASPMode.LAUNCH_LOCKED:
                """
                LAUNCH_LOCKED mode. Not currently implemented.
                Entry: move AVC A mechanism to LAUNCH LOCK state, move AVC B mechanism to LAUNCH LOCK state
                STOP
                """
                pass
            
            case GRASPMode.FREE_FLIGHT:
                """
                FREE_FLIGHT mode. Not currently implemented.
                Entry: move GRAPPLE mechanism to FREE_FLIGHT state
                Exit:
                READY command recieved -> Transitions to READY mode
                """
                # --- Entry/Ongoing Actions ---
                action_complete = self.grapple_controller.free_flight()
                
                # --- Completion-based Transitions ---
                if not action_complete:
                    return
                
                # --- Command-triggered Transitions ---
                if self.GRASP_command == GRASPCommand.READY:
                    self.GRASP_mode = GRASPMode.READY
                    self.GRASP_command = GRASPCommand.NONE
                    self.get_logger().info('Changed GRASP mode to READY.')
                    return
                    
            case GRASPMode.READY:
                """
                READY mode.
                Entry: move GRAPPLE mechanism to OPEN state
                Exit: 
                TRIGGER command received -> Transitions to SOFT_DOCK mode
                """
                
                # --- Entry/Ongoing Actions ---
                action_complete = self.grapple_controller.open()
                
                # --- Completion-based Transitions ---
                if not action_complete:
                    return
                
                # --- Command-triggered Transitions ---
                if self.GRASP_command == GRASPCommand.TRIGGER:
                    self.GRASP_mode = GRASPMode.SOFT_DOCK
                    self.GRASP_command = GRASPCommand.NONE
                    self.get_logger().info('Changed GRASP mode to SOFT_DOCK.')
                    return
            
            case GRASPMode.SOFT_DOCK:
                """
                SOFT_DOCK mode.
                Entry: move GRAPPLE mechanism to SOFT_DOCK state
                Exit: 
                1) GRAPPLE mechanism reaches SOFT_DOCK state -> Transitions to HARD_DOCK mode
                2) CLEARANCE command received -> Transitions to CLEARANCE mode
                """
                # --- Entry/Ongoing Actions ---
                action_complete = self.grapple_controller.soft_dock()
                
                # --- Override/Emergency Exit Conditions (before completion) ---
                if self.GRASP_command == GRASPCommand.CLEARANCE:
                    self.GRASP_mode = GRASPMode.CLEARANCE
                    self.GRASP_command = GRASPCommand.NONE
                    self.get_logger().info('Changed GRASP mode to CLEARANCE.')
                    return
                
                # --- Completion-based Transitions ---
                if action_complete:
                    self.GRASP_mode = GRASPMode.HARD_DOCK
                    self.get_logger().info('Changed GRASP mode to HARD_DOCK.')
                    return
                
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
                # --- Entry/Ongoing Actions ---
                action_complete = self.grapple_controller.hard_dock()
                
                # --- Override/Emergency Exit Conditions (before completion) ---
                if self.GRASP_command == GRASPCommand.CLEARANCE:
                    self.GRASP_mode = GRASPMode.CLEARANCE
                    self.GRASP_command = GRASPCommand.NONE
                    self.get_logger().info('Changed GRASP mode to CLEARANCE.')
                    return
                
                # --- Completion-based Transitions ---
                if not action_complete:
                    return
                
                # --- Command-triggered Transitions (after completion) ---
                if self.GRASP_command == GRASPCommand.RELEASE:
                    self.GRASP_mode = GRASPMode.RELEASE
                    self.GRASP_command = GRASPCommand.NONE
                    self.get_logger().info('Changed GRASP mode to RELEASE.')
                    return
                    
                if self.GRASP_command == GRASPCommand.AVC_A_POS1:
                    self.GRASP_mode = GRASPMode.AVC_A_POS1
                    self.GRASP_command = GRASPCommand.NONE
                    self.get_logger().info('Changed GRASP mode to AVC_A_POS1.')
                    return
                    
                if self.GRASP_command == GRASPCommand.AVC_B_POS1:
                    self.GRASP_mode = GRASPMode.AVC_B_POS1
                    self.GRASP_command = GRASPCommand.NONE
                    self.get_logger().info('Changed GRASP mode to AVC_B_POS1.')
                    return
            
            case GRASPMode.CLEARANCE:
                """
                CLEARANCE mode. (Not currently implemented)
                Entry: move GRAPPLE mechanism to CLEARANCE state
                STOP
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
                # --- Entry/Ongoing Actions ---
                action_complete = self.grapple_controller.clearance()
                
                # --- Completion-based Transitions ---
                if action_complete:
                    # Stays in this mode once clearance is reached
                    return
            
            case GRASPMode.AVC_A_POS1:
                """
                AVC_A_POS1 mode. (Not currently implemented)
                Entry: move AVC A mechanism to AVC_A_POS1 state
                Exit: 
                AVC_A_POS2 command received -> Transitions to AVC_A_POS2 mode
                """
                # TODO: Implement AVC_A_POS1 actions
                pass
                
            case GRASPMode.AVC_A_POS1p5:
                """
                AVC_A_POS1p5 mode. (Not currently implemented)
                Entry: move AVC A mechanism to AVC_A_POS1p5 state
                Exit:
                AVC_A_RETRACT command received -> Transitions to AVC_A_RETRACT mode
                """
                # TODO: Implement AVC_A_POS1p5 actions
                pass
                
            case GRASPMode.AVC_A_POS2:
                """
                AVC_A_POS2 mode. (Not currently implemented)
                Entry: move AVC A mechanism to AVC_A_POS2 state
                Exit:
                AVC_A_POS1p5 command received -> Transitions to AVC_A_POS1p5 mode
                """
                # TODO: Implement AVC_A_POS2 actions
                pass
                
            case GRASPMode.AVC_A_RETRACT:
                """
                AVC_A_RETRACT mode. (Not currently implemented)
                Entry: move AVC A mechanism to HOME state
                Exit:
                AVC A mechanism reaches HOME state -> Transitions to HARD_DOCK mode
                """
                # TODO: Implement AVC_A_RETRACT actions
                pass
                
            case GRASPMode.AVC_B_POS1:
                """
                AVC_B_POS1 mode. (Not currently implemented)
                Entry: move AVC B mechanism to AVC_B_POS1 state
                Exit: 
                AVC_B_POS2 command received -> Transitions to AVC_B_POS2 mode
                """
                # TODO: Implement AVC_B_POS1 actions
                pass
                
            case GRASPMode.AVC_B_POS1p5:
                """
                AVC_B_POS1p5 mode. (Not currently implemented)
                Entry: move AVC B mechanism to AVC_B_POS1p5 state
                Exit:
                AVC_B_RETRACT command received -> Transitions to AVC_B_RETRACT mode
                """
                # TODO: Implement AVC_B_POS1p5 actions
                pass
                
            case GRASPMode.AVC_B_POS2:
                """
                AVC_B_POS2 mode. (Not currently implemented)
                Entry: move AVC B mechanism to AVC_B_POS2 state
                Exit:
                AVC_B_POS1p5 command received -> Transitions to AVC_B_POS1p5 mode
                """
                # TODO: Implement AVC_B_POS2 actions
                pass
                
            case GRASPMode.AVC_B_RETRACT:
                """
                AVC_B_RETRACT mode. (Not currently implemented)
                Entry: move AVC B mechanism to HOME state
                Exit:
                AVC B mechanism reaches HOME state -> Transitions to HARD_DOCK mode
                """
                # TODO: Implement AVC_B_RETRACT actions
                pass
            
            case GRASPMode.HEATER_ON:
                """
                HEATER_ON mode. Not currently implemented.
                """
                # TODO: Implement heater control
                pass
                
            case GRASPMode.HEATER_OFF:
                """
                HEATER_OFF mode. Not currently implemented.
                """
                # TODO: Implement heater control
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