# SCUTTLE_ROS
SCUTTLE ROS

Ros Installation Guide for Rosberry PI Setup by Christian (Resgreen Group) - These are my notes of how to do my setup. It's messy and not for the faint of heart yet.
Notes: This applies to Raspberry Pi 4 devices. I'm testing on Rpi4 8Gb but others should be the same. This setup is different from usual ros due to quirks of rpi and building ros differently.
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
