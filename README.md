# docklab2
Code for running Docklab 2 at OFL

## Setup instructions: 
 - Use Raspberry Pi imager to install Ubuntu desktop 24.04.01 LTS (64 bit)
 - Set hostname "abpX" (where X is a unique identifier below) for air bearing platforms. Use "base" for the hub and "ejection" for the ejection mechanism
   - X = "1": Servicer
   - X = "2": Client
 - Set username and password:
   - username: labpi
   - password: XXXXXXXXXXXXX
 - Set computer name to be the same as hostname
 - Configure wireless LAN for Docklab 2 travel router
 - Install ROS 2 Jazzy Jalisco following ROS 2 documentation here: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#id4
 - Install github if not installed already following guide here: https://gist.github.com/derhuerst/1b15ff4652a867391f03#file-linux-md
 - Create ROS 2 workspace in home directory using "mkdir -p ~/docklab2_ws/src"
 - Navigate to /docklab2_ws/src and clone docklab2 repository into folder
 - Navigate back to /docklab2_ws and run "colcon build --symlink-install --executor sequential" to build the overlay
 - Close the terminal used to build the overlay before running
 - Source the overlay by running "source install/setup.bash" in a new terminal before running 
 
# Network Setup Instructions
 - Connect device to AC750 Wi-Fi Travel Router (TP-Link_8AC3 not TP-Link_8AC3_5G)
 - Login to router at http://tplinkwifi.net and configure access as required
 - Note down IP address and subnet mask of router on screen
 - Find network interface name for device by running command "ip addr". Ethernet connections will start with the letter e, wireless connections will start with the letter w. 
 - Write the network configuration to the device using "sudo nano /etc/netplan/docklab-network-config.yaml" (see config below)
 - Apply changes using "sudo netplan generate" and "sudo netplan apply"
"""
network:
    version: 2
    renderer: NetworkManager
    
    wifis: 
        wlan0:
            dhcp4: no
            addresses: [192.168.0.XXX/24]
            gateway4: 192.168.0.1
            nameservers:
                addresses: [192.168.0.1]
            access-points:
                "TP-Link_8AC3":
                    password: "YYYYYYYY"
"""
XXX is the port of the device: 
200 is the base station
201 is abp1
202 is abp2
YYYYYYYY is the password for the router

 - Setup ssh using "sudo apt install openssh-server"

# Python Setup Instructions
 - Install venv for python3 by running "sudo apt install python3.12-venv"
 - Create virtual environment for running python in home directory called "py_env" by running "python3 -m venv py_env"

# Bashrc setup 
 - Add "source /opt/ros/jazzy/setup.bash" to .bashrc file in home directory to source ROS on start up of the terminal
 - Add "source py_env/bin/activate" to .bashrc file in home directory to source python environment on start up of the terminal 
 
# Serial setup instructions
 - Execute "sudo chmod 666 /dev/ttyACM0" to enable opening serial port ttyACM0. This port may change if there is need to use serial comms with hardware other than the Teensy 4.1 

 # GPIO setup instructions
  - To enable using GPIO over ssh add user to dialout group as below:
"""
sudo apt install rpi.gpio-common
sudo adduser "${USER}" dialout
sudo reboot

# Solo Motor Controller Setup 
 - Install SoloPy here: https://github.com/Solo-FL/SoloPy.git
 - AVC device address: 0
 - Grapple device address: 1

# LabJack Setup Instructions
 - Follow instructions for downloading LabJack for python here: https://support.labjack.com/docs/python-for-ljm-windows-mac-linux
    - Install AArch64 installer for LabJack LJM Library from here: https://files.labjack.com/installers/LJM/Linux/AArch64/beta/LabJack-LJM_2025-01-10.zip (see INSTALL.md for instructions)
    - Install labjack-ljm to py_env environment using "pip install labjack-ljm" (make sure py_env environment is active during this step)

## Mocap
https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack.git
- follow instructions
- connect laptot to mocap laptop with ethernet. (check if you can do this wirelessly)
- set motive to unicast with ip address of ros2 laptop 
- change config folder to ip address of local and server
- soruce wkspace
- stream

- this package doesnt work on a Raspberry Pis' or on anything older than humble
## Docs: 

# ABPX:
Services: 
relay_server3 - server for services for changing state of relays on RPi Relay Board with 3 relays (https://thepihut.com/products/raspberry-pi-relay-board)
relay_server6 - server for services for changing state of relays on sb components PiRelay 6 Channel (https://thepihut.com/products/pirelay-6)

Nodes: 
GRASP_node - node for communicating with GRASP MDE and updating GRASP state

Topics: 

Services:

# Running Docklab 2: 
Setup:

On Labtop:
 - Start Motive on Labtop
 - If necessary calibrate camera setup
 - If necessary select assets 
 - Enable streaming of data to base station over Ethernet
On base station: 
 - Open terminal and navigate to mocap4r2_ws
 - Source the workspace using "source install/setup.bash"
 - Launch the Optitrack system using "ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py"
 - Open a new terminal and navigate to mocap4r2_ws
 - Source the workspace using "source install/setup.bash"
 - Transition the driver node to active using "ros2 lifecycle set /mocap4r2_optitrack_driver_node activate"
 - Open a new terminal and navigate to mocap4r2_ws
 - Source the workspace using "source install/setup.bash"
 - Run mocap_sub node using "ros2 run mocap_sub mocap_sub"
On ABPX:
 - Connect to ABPX over ssh from base station "ssh labpi@abpX.local"
 - Change directory to docklab2_ws and source the workspace using "source install/setup.bash"
 - Launch the node for the platform using "ros2 launch docklab2 docklab2_abpX.launch.py"

Running: 

On base station: 
 - Open terminal and run rqt using "rqt"
 - Send commands to terminal manually over topic publisher or service tool.
 - To run complex commands:
 - Change directory to docklab2_ws and source the workspace using "source install/setup.bash"
 - Run the base control node using "ros2 run docklab2 base_control"

## Notes: 
 - colcon build command can crash the Raspberry Pi when running, use: colcon build --symlink-install --executor sequential (https://answers.ros.org/question/404536/colcon-build-fails-on-ros2-tutorials/)
