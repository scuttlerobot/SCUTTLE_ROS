Notes on robot_upstart setup and configuration.

This has been tested on Ubuntu 18.04 for Jetson Nano and Ros Melodic and also Raspi 4 with 18.04 ubuntu Melodic.

Note: This is done using the sudo apt-get install ros-$DISTRO-robot-upstart not using install generator but that method should work as well!

Explanation and overview of steps.
Robot Upstart is a ROS package that allows you to select a launch file, xacro and other things to run at boot on your device as a service. My example here does not use a Xacro (this will be a future addition), but it does use a bring_up.launch file and sets some environment variables in the my_robot_ros_start script. 
1. Make sure you have followed the guide here: https://github.com/scuttlerobot/SCUTTLE_ROS/tree/main/src and create the scuttle, scuttle_description and have downloaded the rp_lidar_ros package.
2. Download the robot_upstart package (apt-get method)
```
sudo apt-get install ros-$DISTRO-robot-upstart
```
3. We need to run the install script for robot_upstart
```
rosrun robot_upstart install scuttle/launch/scuttle_bringup.launch --job my_robot_ros --symlink
```
- You may need to enter the sudo password during the install process, after restart the systemctl daemon
```
sudo systemctl daemon-reload
```
4. Once, installation is complete we need to modify a few things. Your changes will be specific to your setup. In my example below the Scuttle Robot is the Master and my laptop that runs rviz is able to connect via hostname.
```
(nano version)
sudo nano /usr/sbin/my_robot_ros-start
or (vim version)
sudo vim /usr/sbin/my_robot_ros-start
```
- We need to edit our file to have the following 2 lines reflect the hostname of your device. (Cool Snippet: Ubuntu comes with the Avahi Daemon and lets you find your robot with hostname via scuttle.local or whatever your hostname is. We will use this to our advantage and use our hostname.local as our ROS_MASTER_URI and ROS_HOSTNAME.
```
export ROS_HOSTNAME=<hostname goes here>.local
export ROS_MASTER_URI=http://<hostname goes here>.local:11311 
```
5. Make sure your scuttle_bringup.launch is not currently running. Then we will test the service.
```
sudo systemctl start my_robot_ros
```

6. Check with either rostopic list or rostopic echo <topic name> to make sure all is working as expected. You can stop the service with
```
sudo systemctl stop my_robot_ros
```
  
- now this should run your launch file on boot, start roscore at your hostname and bring up the rp_lidar and cmd_vel_subscriber.
  
