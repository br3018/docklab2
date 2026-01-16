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

# Import GraspPy module
from GraspPy import GrappleController, AVCController

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
                ('grasp_model', rclpy.Parameter.Type.STRING),

                ('feedback_params.frequency', rclpy.Parameter.Type.DOUBLE),

                ('state_machine_params.frequency', rclpy.Parameter.Type.DOUBLE)
            ])
        
        # Find available serial ports
        available_ports = self.find_available_ports()

        # Get GRASP model
        grasp_model = self.get_parameter('grasp_model').get_parameter_value().string_value

        # Initialize motor controllers
        self.get_logger().debug('Initializing motor controllers')
        
        # Initialize Grapple Controller
        self.grapple_controller = GrappleController(grasp_model, self.get_logger(), available_ports)

        # Initialize AVC A Controller
        self.avc_a_controller = AVCController('avc_a', grasp_model, self.get_logger(), available_ports)

        # Initialize AVC B Controller
        self.avc_b_controller = AVCController('avc_b', grasp_model, self.get_logger(), available_ports)

        # Set up a function that runs the state machine
        self.state_machine_frequency = self.get_parameter('state_machine_params.frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / self.state_machine_frequency, self.GRASP_state_machine)

        # Set up a function that publishes data at a set frequency
        self.feedback_frequency = self.get_parameter('feedback_params.frequency').get_parameter_value().double_value
        self.feedback_timer = self.create_timer(1.0 / self.feedback_frequency, self.publish_data)

        # Add GRASP flags subscribers
        self.subscription = self.create_subscription(String, 'GRASP_flags', self.GRASP_external_flags, 10) #QoS arbitrarily set at 10

        
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