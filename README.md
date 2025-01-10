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

# Python Setup Instructions
 - Install venv for python3 by running "sudo apt install python3.12-venv"
 - Create virtual environment for running python in home directory called "py_env" by running "python3 -m venv py_env"

# Bashrc setup 
 - Add "source /opt/ros/jazzy/setup.bash" to .bashrc file in home directory to source ROS on start up of the terminal
 - Add "source py_env/bin/activate" to .bashrc file in home directory to source python environment on start up of the terminal 
 
# LabJack Setup Instructions
 - Follow instructions for downloading LabJack for python here: https://support.labjack.com/docs/python-for-ljm-windows-mac-linux
    - Install Linuxx64 version of LJM software here: https://support.labjack.com/docs/ljm-software-installer-downloads-t4-t7-t8-digit#LJMSoftwareInstallerDownloads-T4,T7,T8,Digit-Linuxx64LJMSoftwareInstallerDownloads
    - Install labjack-ljm to py_env environment using "pip install labjack-ljm" (make sure py_env environment is active during this step)
 
## Run instructions: 
 - Launch abp1 code using command ros2 launch docklab2 docklab2_abp1.launch.py
 
## Docs: 
# ABPX:
Services: 
relay_server3 - server for services for changing state of relays on RPi Relay Board with 3 relays (https://thepihut.com/products/raspberry-pi-relay-board)
relay_server6 - server for services for changing state of relays on sb components PiRelay 6 Channel (https://thepihut.com/products/pirelay-6)

Nodes: 

Topics: 

Services:

## Notes: 
 - colcon build command can crash the Raspberry Pi when running, use: colcon build --symlink-install --executor sequential (https://answers.ros.org/question/404536/colcon-build-fails-on-ros2-tutorials/)
