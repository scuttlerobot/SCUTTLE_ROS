# SCUTTLE_ROS
SCUTTLE ROS

Ros Installation Guide for Rosberry PI Setup by Christian (Resgreen Group) - These are my notes of how to do my setup. It's messy and not for the faint of heart yet.
Notes: This applies to Raspberry Pi 4 devices. I'm testing on Rpi4 8Gb but others should be the same. This setup is different from usual ros due to quirks of rpi and building ros differently.

PREREQ: This requires raspbian buster (you can get this in raspi imager tool for windows in gui or lite form)

1. Setup ROS Repositories
- Install the repo key for ros and update apt
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get upgrade
```
- Install dependencies needed for ROS install and setup
```
sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
```
- Setup Rosdep
```
sudo rosdep init
rosdep update
```
2. Setup the ros_catkin_ws
- Make a ros_catkin_ws to compile the ros packages. This is different from the typical sudo apt-get install ros-$DISTRO-blah-blah-blah method, but better for RPI according to rosberry setup.
```
mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws
```
- Here we pick out our packages to install (if you want to install new ones later you can and I show that below!). For my example below we will install ros_comm only to start!
```
rosinstall_generator ros_comm --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall
wstool init src melodic-ros_comm-wet.rosinstall
cd ~/ros_catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic
source /opt/ros/melodic/setup.bash
```
3. Add more standard packages now that the base is installed
- Pick out new packages to install, but you MUST include the old ones as well! We are gonna add teleop_twist_keyboard, teleop_twist_joy, ros_control, joystick_drivers, robot_state_publisher and joint_state_publisher
```
rosinstall_generator ros_comm ros_control joystick_drivers robot_state_publisher joint_state_publisher teleop_twist_keyboard teleop_twist_joy --rosdistro melodic --deps --wet-only --tar > melodic-custom_ros.rosinstall 
wstool merge -t src melodic-custom_ros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro melodic -y -r --os=debian:buster
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic
source /opt/ros/melodic/setup.bash
```

*Optional Scripting for Auto Start*
Assuming you made a catkin_ws (NOT the same as ros_catkin_ws) and followed the setup for the scuttle and two_wheel_driver packages.
1. Script in .bashrc to set enviroment variables based on your robots IP address. This is to point all ros nodes to roscore running on your robot. If you want to access the robot with a remote laptop you would connect to the robots ROS_MASTER_URI. The script below sets the IP for roscore on the robot.
```
cd
nano .bashrc
# scroll way way down to the bottom and add the following
source /opt/ros/melodic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
begin="http://"
ip=$(hostname -I)
ip2=${ip::-1}
port=":11311"
export ROS_MASTER_URI=$begin$ip2$port
export ROS_IP=$ip2
```
exit the terminal and reconnect via ssh. To check if this worked run the following to see your Master URI and IP variables
```
echo $ROS_MASTER_URI
echo $ROS_IP
```
both should have the same IP address but look different. One is a full web address with a port for the ros master. The other is just your devices IP.
To connect with a remote laptop on the same network you would set the laptops ROS_MASTER_URI the same as the robots, but set the ROS_IP as the laptops ip.

2. Script to set ROS_MASTER_URI and ROS_IP and launch roscore on boot
Notes: This creates a service in /etc/init.d/
```
sudo nano /etc/init.d/rosstart.sh
# Add the following content to the file

#!/bin/bash
begin="http://"
ip=$(hostname -I)
ip2=${ip::-1}
port=":11311"
export ROS_MASTER_URI=$begin$ip2$port
export ROS_IP=$ip2
roscore
```
Next we need to make the OS aware of the new script and make it executable
```
chmod +x /etc/init.d/rosstart.sh
sudo update-rc.d rosstart.sh defaults
```

test it with a reboot!
