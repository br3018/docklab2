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

# Import Python libraries 
import SoloPy as solo
# if solopy is not found in ROS2, need to run this before 
# export PYTHONPATH=/home/labpi/py_env/lib/python3.12/site-packages:$PYTHONPATH
import serial
import time



class GRASPNode(Node):
    # Constructor
    def __init__(self):
        # Set node name 
        super().__init__('GRASP_node')
        # Set up motor controller instances

        
        self.avc_Solo     = self.avc_motor_init()
        #self.grapple_Solo = self.grapple_motor_init()
        
        #set up function that runs constatntly monitoring the state machine
        self.timer = self.create_timer(2, self.GRASP_state_machine)
        self.timer = self.create_timer(1, self.periodic_feedback)
        
        # Add serial close service
        #self.srv = self.create_service(Trigger, 'grapple_init', self.grapple_init_callback)
        #self.srv = self.create_service(Trigger, 'grapple_init', self.avc_init_callback)


        # Add GRASP state subscriberself.get_logger().info(f"   - Position counts feedback: {self.avc_Solo.get_position_counts_feedback()[0]}.")
        self.subscription = self.create_subscription(String, 'GRASP_flags', self.GRASP_external_flags, 10) #QoS arbitrarily set at 10
        self.subscription = self.create_subscription(Float64,'/avc_motor/position_cmd',self.avc_motor_callback,10)
        self.subscription = self.create_subscription(Float64,'/avc_motor/avc_feedback_cmd',self.avc_feedback,10)
        self.subscription = self.create_subscription(Float64,'/avc_motor/full_rotation_cmd',self.avc_motor_full_turn,10)
        self.subscription # prevent unused variable error
        
        self.publication = self.create_publisher(String, 'motor_state', 10)
        self.publication = self.create_publisher(String, 'motor_feedback', 10)
        
        self.get_logger().info('GRASP_node initiated with a periodic state machine, and state set to HOME.')
        
        
        self.current_state = 'HOME'
        self.current_pos = self.avc_Solo.get_position_counts_feedback()[0]
        
    def grapple_motor_init(self):
        #Connect with the motor
        
        grapple_Solo = solo.SoloMotorControllerUart("/dev/ttyACM1") #This port connection worked.
        print(f"Device address: {grapple_Solo.get_device_address()[0]}")
        
        if grapple_Solo.connect():
                
            self.get_logger().info(f"Connected to GRAPPLE motor")
            
            print("-------PARAMETERS --------")
            print(f"    GRAPPPLE Board Temperature: {str(grapple_Solo.get_board_temperature()[0])} degrees")
            print("    The position controller gains for the AVC motor are:")
            print("        Kp = " + str(grapple_Solo.get_position_controller_kp()[0] ) )
            print("        Ki = " + str(grapple_Solo.get_position_controller_ki()[0] ) )
            print("    The velocity controller gains are:")
            print("        Kp = " + str(grapple_Solo.get_speed_controller_kp()[0] ) )
            print("        Ki = " + str(grapple_Solo.get_speed_controller_ki()[0] ) )
            print("    The current controller gains are:")
            print("        Kp = " + str(grapple_Solo.get_current_controller_kp()[0] ) )
            print("        Ki = " + str(grapple_Solo.get_current_controller_ki()[0] ) )
            print("    Control mode is: " + str(grapple_Solo.get_control_mode()[0] ) )
            print("    Speed Limit is: " + str(grapple_Solo.get_speed_limit()[0] ) )
            
            # Set tuning paramters to GRAPPLE motor 
            print("    Control mode is: " + str(grapple_Solo.get_control_mode()))
        else:
            self.get_logger().warning('Failed to establish connection with GRAPPLE Solo Motor Controller')
        return grapple_Solo
        
    #def grapple_motor_callback(self, msg):
    #    position = msg.data
    #    self.get_logger().info(f"Connected to GRAPPLE motor")
    #    self.grapple_Solo.set_position_reference_deg(position)
    
    def avc_motor_init(self):
    
        #Connect with the motor
        
        avc_Solo = solo.SoloMotorControllerUart("/dev/ttyACM0") #This port connection worked.
        print(f"AVC device address: {avc_Solo.get_device_address()[0]}")
        
        print("--------------- AVC MOTOR ------------------")
        print("---- Position Reference: "       + str(avc_Solo.get_position_reference()[0]) )
        print("---- Position Counts feedback: " + str(avc_Solo.get_position_counts_feedback()[0]) )
        #print("Is the motor enabled: " + str(avc_Solo.is_drive_enabled() ))
        
        if avc_Solo.connect():
            self.get_logger().info(f"Connected to AVC motor")
            
            # Initial Configuration of the device and the Motor
            avc_Solo.set_output_pwm_frequency_khz(20)              # Desired Switching or PWM Frequency at Output
            avc_Solo.set_current_limit(3)                          # Current Limit of the Motor
            avc_Solo.set_motor_poles_counts(4)                     # Motor's Number of Poles
            avc_Solo.set_command_mode(solo.CommandMode.DIGITAL)    # 
            avc_Solo.set_motor_type(solo.MotorType.BLDC_PMSM)
            avc_Solo.set_feedback_control_mode(solo.FeedbackControlMode.HALL_SENSORS)
            avc_Solo.set_speed_controller_kp(0.0099868)            # Speed controller Kp
            avc_Solo.set_speed_controller_ki(0.0029907)            # Speed controller Ki
            avc_Solo.set_position_controller_kp(2)                 # Position controller Kp
            avc_Solo.set_position_controller_ki(0.0099868)         # Position controller Ki
            avc_Solo.set_control_mode(solo.ControlMode.POSITION_MODE)
            avc_Solo.set_speed_limit(4000)                         # Desired Speed Limit[RPM].Check if 3000 is good
            print("-------PARAMETERS --------")
            print(f"    AVC Board Temperature: {str(avc_Solo.get_board_temperature()[0])} degrees")
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
        else:
            self.get_logger().warning('Failed to establish connection with AVC Solo Motor Controller')
        return avc_Solo
        
    def avc_motor_full_turn(self, msg):
        position = msg.data
        init_pos = self.avc_Solo.get_position_counts_feedback()[0]
        self.get_logger().info(f"Initial Position: {init_pos}.")
        ref_pos = init_pos + 2352
        self.get_logger().info(f"Commanding now avc_motor to rotate a full turn. From pos {init_pos} to {ref_pos}")
        #self.avc_Solo.set_position_reference(ref_pos)
        time.sleep(10)
        self.get_logger().info(f"Rotation completed: Current pos: {self.avc_Solo.get_position_counts_feedback()[0]}.")
        
    def avc_motor_callback(self, msg):
        position = msg.data
        
        self.get_logger().info(f"Position counts feedback: {self.avc_Solo.get_position_counts_feedback()[0]}.")
        self.get_logger().info(f"Commanding now avc_motor to rotate to position {position}")
        self.avc_Solo.set_position_reference(position)
        self.get_logger().info(f"Position counts feedback: {self.avc_Solo.get_position_counts_feedback()[0]}.")
        
    def avc_feedback(self, msg):
        position = msg.data
        
        self.get_logger().info(f"Getting feedback from the motor and board: ")
        self.get_logger().info(f"   - Position Reference: {      self.avc_Solo.get_position_reference()[0]}.")
        self.get_logger().info(f"   - Position counts feedback: {self.avc_Solo.get_position_counts_feedback()[0]}.")
        self.get_logger().info(f"   - Speed feedback: {          self.avc_Solo.get_speed_feedback()[0]}.")

    def serial_disconnect_callback(self, request, response):
        # Closing serial port
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
        
    def GRASP_external_flags(self,msg):
        flag = msg.data   
        
        print('reached here 1')
        
        match flag:
        
            case 'GO_OPEN':
                
                print('reached here 2')
                if self.current_state != 'HOME':
                    # There must be an error. Previous state has to be HOME.
                    # We should abort
                    self.get_logger().warning("Failed to change mode to 'OPEN' because previous state isn't HOME. ")
                    return
        
                # Set position reference to 5000.
                open_pos_delta = 2352
                
                initial_pos = self.avc_Solo.get_position_counts_feedback()[0];
                self.target_pos  = initial_pos + open_pos_delta
                
                self.current_pos = self.avc_Solo.get_position_counts_feedback()[0]
                
                self.get_logger().info(f"Position counts feedback: {self.avc_Solo.get_position_counts_feedback()[0]}.")
                self.get_logger().info(f"Commanding now avc_motor to rotate to position {self.target_pos}")
                #self.avc_Solo.set_position_reference(self.target_pos)
                
                self.current_state = 'OPENING'
                
            case 'GO_CAPTURE':
            
                if self.current_state != 'OPEN':
                    # There must be an error. Previous state has to be HOME.
                    # We should abort
                    self.get_logger().warning("Failed to change mode to 'CAPTURE' because previous state isn't OPEN. ")
                    return
        
                # Set position reference to 5000.
                capture_pos_delta = -1000
                
                initial_pos = self.avc_Solo.get_position_counts_feedback()[0];
                self.target_pos  = initial_pos + capture_pos_delta
                
                self.get_logger().info(f"Position counts feedback: {self.avc_Solo.get_position_counts_feedback()[0]}.")
                self.get_logger().info(f"Commanding now avc_motor to rotate to position {self.target_pos}")
                #self.avc_Solo.set_position_reference(self.target_pos)
                
                self.current_state = 'CAPTURING'
    
    def periodic_feedback(self):
    
        try:
            pos_ref_fb        = self.avc_Solo.get_position_reference()[0]
            pos_counts_fb     = self.avc_Solo.get_position_counts_feedback()[0]
            speed_feedback_fb = self.avc_Solo.get_speed_feedback()[0]
            #self.get_logger().info(f"Getting feedback from the motor and board: ")
            #self.get_logger().info(f"   - Position ref:    {pos_ref_fb}.")
            #self.get_logger().info(f"   - Position counts: {pos_counts_fb}.")
            #self.get_logger().info(f"   - Speed feedback:  {speed_feedback_fb}.")
            
            msg = String()
            msg.data = f"State: {self.current_state}, Pos ref: {pos_ref_fb}, Pos counts: {pos_counts_fb},Speed: {speed_feedback_fb}"
            self.publication.publish(msg)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to read motor feedback: {e}")
            
            
    def GRASP_state_machine(self):
    
        # Checking GRASP state and calling relevant code.
        match self.current_state:
        
            case "HOME":
                # We do nothing in this state. We are
                state = "1"
                #self.get_logger().info('GRASP state is: HOME')
                #self.get_logger().info(f"   - GRAPPLE motor is at position: {self.avc_Solo.get_position_counts_feedback()[0]}.")
                #self.ser.write(state.encode())
                
                # We run this mode continuosly and wait for the flag "GO_OPEN"
                
            case "OPENING":
                state = "2"
                # This mode changes motor position to have GRASP fully opened.
                # This mean we came from a HOME position 
                self.current_pos = self.avc_Solo.get_position_counts_feedback()[0]
                
                #self.get_logger().info('GRASP state is: OPENING')
                #print(f"    Current pos: {self.current_pos}")
                #print(f"    Target pos: {self.target_pos}")
                
                
                if self.current_pos < self.target_pos:
                    #self.current_pos = self.current_pos + 1000
                    # Waiting for the position to change
                    self.get_logger().info(f"   Performing rotation to pos {self.target_pos}, Current pos: {self.current_pos}.")
                else:
                    self.get_logger().info(f"   Rotation for opening completed: Current pos: {self.current_pos}.")
                    self.current_state = 'OPEN'
                    
            case "OPEN":
                state = "3"
                self.get_logger().info('GRASP state is: OPEN')
                self.get_logger().info(f"   - GRAPPLE motor is at position: {self.avc_Solo.get_position_counts_feedback()[0]}.")
                
            case "CAPTURING":
                state = "4"
                self.current_pos = self.avc_Solo.get_position_counts_feedback()[0]
                self.get_logger().info('GRASP state is: CAPTURING')
                
                if self.current_pos > self.target_pos:
                    #self.current_pos = self.current_pos - 100
                    self.get_logger().info(f"   Performing rotation to pos {self.target_pos}, Current pos: {self.current_pos}.")
                else:
                    self.get_logger().info(f"   Rotation for docking completed: Current pos: {self.current_pos}.")
                    self.current_state = 'SOFT_DOCK'
                    
            case "SOFT_DOCK":
                state = "5"
                self.get_logger().info('GRASP state is: SOFT DOCK')
                self.get_logger().info(f"   - GRAPPLE motor is at position: {self.avc_Solo.get_position_counts_feedback()[0]}.")
                
                
            case "RETRACT":
                state = "6"
                self.get_logger().info('Updating GRASP state to RETRACT')
                self.ser.write(state.encode())
            case "RELEASE":
                state = "7"
                self.get_logger().info('Updating GRASP state to RELEASE')
                #self.ser.write(state.encode())
            case "RUN":
                state = "8"
                self.get_logger().info('Updating GRASP state to RUN')
                #self.ser.write(state.encode())
            case "MORE":
                state = "9"
                self.get_logger().info('Updating GRASP state to MORE')
                self.ser.write(state.encode())
            case "LESS":
                state = "10"
                self.get_logger().info('Updating GRASP state to LESS')
                self.ser.write(state.encode())
            case "AVC_HOME":
                state = "11"
                self.get_logger().info('Updating GRASP state to AVC_HOME')
                self.ser.write(state.encode())
            case "FORWARD":
                state = "12"
                self.get_logger().info('Updating GRASP state to AVC_FORWARD')
                self.ser.write(state.encode())
            case "BACK":
                state = "11"
                self.get_logger().info('Updating GRASP state to AVC_BACK')
                self.ser.write(state.encode())
            case _: # Catch invalid command 
                self.get_logger().warning('Unknown GRASP state recieved')
                
            


def main():
    rclpy.init()
    GRASP_node = GRASPNode()
    rclpy.spin(GRASP_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
