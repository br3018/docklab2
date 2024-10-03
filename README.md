# docklab2
Code for running Docklab 2 at OFL

## Setup instructions: 
 - Use Raspberry Pi imager to install Ubuntu desktop 24.04.01 LTS (64 bit)
 - Set hostname abpX (where X is a unique identifier below)
   - X = 1: Servicer
   - X = 2: Client
 - Set username and password:
   - username: labpi
   - password: XXXXXXXXXXXXX
 - Configure wireless LAN for docklab-2 travel router
 - Install ROS 2 Jazzy Jalisco following ROS 2 documentation here: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#id4
 - Install github if not installed already following guide here: https://gist.github.com/derhuerst/1b15ff4652a867391f03#file-linux-md
 - Create ROS 2 workspace in home directory using "mkdir -p ~/ros2_ws/src"
 - Navigate to /ros2_ws/src and clone docklab-2 repository into folder
 - Navigate back to /ros2_ws and run "colcon build --symlink-install --executor sequential" to build the overlay
 - Close the terminal used to build the overlay before running
 - Source the overlay by running "source install/setup.bash" in a new terminal before running 

## Notes: 
 - colcon build command can crash the Raspberry Pi when running, use: colcon build --symlink-install --executor sequential (https://answers.ros.org/question/404536/colcon-build-fails-on-ros2-tutorials/)
