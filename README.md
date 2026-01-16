# docklab2
Code for running Docklab 2 at OFL

## GRASP software setup
Install GraspPy module following instructions in git repo: https://github.com/OFI-UK/GraspPy

## Setup instructions for air bearing platforms: 
 - Use Raspberry Pi imager to install Ubuntu desktop 24.04.01 LTS (64 bit) for ROS 2 Jazzy or Ubuntu 22.04.5 LTS (64 bit) for ROS 2 Humble
 - Set hostname "abpX" (where X is a unique identifier below) for air bearing platforms. Use "base" for the hub and "ejection" for the ejection mechanism
   - X = "1": Servicer
   - X = "2": Client
 - Set username and password:
   - username: labpi (for Raspberry Pi)
   - username: labpc (for desktop)
   - password: XXXXXXXXXXXXX
 - Set computer name to be the same as hostname
 - Configure wireless LAN for Docklab 2 travel router
 - Install ROS 2 Jazzy Jalisco or ROS 2 Humble following ROS 2 documentation here: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#id4 or here https://docs.ros.org/en/humble/Installation.html
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

# Python Setup Instructions Ubuntu 24
 - Ubuntu 24.04.01 LTS requires packages to be managed in a virtual environment with pip
 - Ubuntu 22.04.5 LTS does not require virtual environments for the pip package manager to work
 - When working with Ubuntu 24, follow the following instructions for setting up the virtual environment and ensure any new packages are installed in the same environment
 - Install venv for python3 by running "sudo apt install python3.12-venv"
 - Create virtual environment for running python in home directory called "py_env" by running "python3 -m venv py_env"
  - Add "source py_env/bin/activate" to .bashrc file in home directory to source python environment on start up of the terminal 

# Bashrc setup 
 - Add "source /opt/ros/jazzy/setup.bash" for ROS 2 Jazzy or "source /opt/ros/humble/setup.bash" to .bashrc file in home directory to source ROS 2 on start up of the terminal
 
# Serial setup instructions
 - To enable using serial ports add user to dialout group as below:
 """
sudo adduser "${USER}" dialout
sudo reboot

# Raspberry Pi GPIO setup instructions
  - To enable using GPIO over ssh add user to dialout group as before, install the Raspberry Pi gpio packages:
"""
sudo apt install rpi.gpio-common
sudo adduser "${USER}" dialout
sudo reboot

# Solo Motor Controller Setup 
 - AVC A device address: 3
 - AVC B device address: 4
 - Grapple device address: 7

# LabJack Setup Instructions
 - Follow instructions for downloading LabJack for python here: https://support.labjack.com/docs/python-for-ljm-windows-mac-linux
    - For Raspberry Pi, install AArch64 installer for LabJack LJM Library from here: https://files.labjack.com/installers/LJM/Linux/AArch64/beta/LabJack-LJM_2025-01-10.zip (see INSTALL.md for instructions)
    - Install labjack-ljm to py_env environment using "pip install labjack-ljm" 

# Mocap Setup Instructions

## Network setup

- Connect mocap laptop and base station over the same network (wired preferred)
- Configure static IP address on mocap laptop. 
  - Set IP assignment to manual 
  - Set IPv4 address to what you want (192.168.1.61 used for ISAM facility testing)
  - Set subnet mask to 255.255.255.0 (most cases using IPv4 this does not need to be different)
  - Leave default gateway empty
- Configure static IP address on base station.
  - Set IPv4 method to manual
  - Set IPv4 address to what you want (192.168.1.60 used for ISAM facility testing)
  - Set subnet mask to 255.255.255.0 (see above note)
  - Leave default gateway empty

## Software setup

- To install motion capture packages into the workspace follow instructions linked below:
https://github.com/br3018/mocap4ros2_optitrack
- Connect base station to mocap laptop over ethernet or wireless connection.
  - If connecting wirelessly be aware latency can be high
- Launch and setup motive performing calibrations as necessary 
- Set motive to unicast with local interface of mocap laptop on shared network with base station
- Assign GRASP asset streaming ID of 1 and RAFTI asset streaming ID of 2
- On base station, setup optitrack configuration at docklab2_ws/src/mocap4ros2_optitrack/mocap4r2_optitrack_driver/config/mocap4r2_optitrack_driver_params.yaml
  - Set server address as mocap laptop IP address on shared network
  - Set local address as base station IP address on shared network
  - Set other parameters to be the same as in the motive streaming menu where available
  - Do not change any other parameters
 
- To launch the optitrack system connection follow the instructions in the github repository to launch each node manually or use:
"""
ros2 launch mocap-transforms optitrack_bringup.launch.py
"""
In a seperate terminal: 
"""
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
"""
- Successful launch will display the below:
"""
[mocap4r2_optitrack_driver_main-1] [INFO] [1750870175.394628061] [mocap4r2_optitrack_driver_node]: Trying to connect to Optitrack NatNET SDK at 192.168.0.100 ...
[mocap4r2_optitrack_driver_main-1] [INFO] [1750870175.504141622] [mocap4r2_optitrack_driver_node]: ... connected!
[mocap4r2_optitrack_driver_main-1] [INFO] [1750870175.507230073] [mocap4r2_optitrack_driver_node]: 
[mocap4r2_optitrack_driver_main-1] [Client] Server application info:
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_main-1] [INFO] [1750870175.507317156] [mocap4r2_optitrack_driver_node]: Application: Motive (ver. 3.1.3.1)
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_main-1] [INFO] [1750870175.507342386] [mocap4r2_optitrack_driver_node]: NatNet Version: 4.1.0.0
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_main-1] [INFO] [1750870175.507361388] [mocap4r2_optitrack_driver_node]: Client IP:192.168.0.200
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_main-1] [INFO] [1750870175.507378546] [mocap4r2_optitrack_driver_node]: Server IP:192.168.0.100
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_main-1] [INFO] [1750870175.507422668] [mocap4r2_optitrack_driver_node]: Server Name:
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_main-1] [INFO] [1750870175.510198551] [mocap4r2_optitrack_driver_node]: Mocap Framerate : 240.00
[mocap4r2_optitrack_driver_main-1] 
[mocap4r2_optitrack_driver_main-1] [INFO] [1750870175.510289276] [mocap4r2_optitrack_driver_node]: Configured!
"""
- This package doesnt work on a Raspberry Pis' or on anything older than humble

## Docs: 

# ABPX:
Nodes: 
GRASP_node - GRASP state machine
labjack_node - node for communicating with LabJack devices for datalogging
base_node - node for controlling docklab 2
relay_server3 - server for services for changing state of relays on RPi Relay Board with 3 relays (https://thepihut.com/products/raspberry-pi-relay-board)
relay_server6 - server for services for changing state of relays on sb components PiRelay 6 Channel (https://thepihut.com/products/pirelay-6)

# Running Docklab 2: 
Setup:

On labtop and base station: 
 - Complete motion capture setup as described above
On ABPX:
 - Connect to ABPX over ssh from base station "ssh labpi@abpX.local"
 - Change directory to docklab2_ws and source the workspace using "source install/setup.bash"
 - Launch the node for the platform using "ros2 launch docklab2 docklab2_abpX.launch.py"

Running: 

On base station: 
 - Open terminal and run rqt using "rqt"
 - Send commands to terminal manually over topic publisher or service tool.
 - Plot data using plotjuggler "ros2 run plotjuggler plotjuggler"
 - To run complex commands:
 - Change directory to docklab2_ws and source the workspace using "source install/setup.bash"
 - Run the base control node using "ros2 run docklab2 base_control"


## Notes: 
 - colcon build command can crash the Raspberry Pi when running, use: colcon build --symlink-install --executor sequential (https://answers.ros.org/question/404536/colcon-build-fails-on-ros2-tutorials/)
