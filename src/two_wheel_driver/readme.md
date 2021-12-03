Notes from Christian: Not final and just for basic guidance

This is a package for controlling the motors on scuttle and also reading basic encoder data

This is very messy and just an experiment.

Prereqs: This was made for ros Kinetic but seems compatible with Melodic
Installed: ROS Melodic/Kinetic on some flavor of ubuntu to match (Server 18.04 or Ubuqitity Robotics image Kinetic-Only)
Scuttle: Wired via the Wiring Guide (May be wrong so be careful!)
	This includes encoders at addresses x40 and x41 respectively
ROS: Already created a catkin workspace!

Packages needed from ros for teleop driving
teleop_twist_joystick or teleop_twist_keyboard - Path differs here, Ill give keyboard instructions as that is easier
this is installed with the code below:
```
sudo apt-get install ros-$ROS_DISTRO-teleop-twist-keyboard 
```
Explaining the $ROS_DISTRO variable:
	You can use ROS environment variables to include stuff for you. In this case its grabbing your ros distro (melodic or kinetic) and then installing the correct version of teleop_twist_keyboard for you.

Install RPi.GPIO and numpy if not already done:
```
pip install RPi.GPIO
sudo apt-get install python-numpy
```
Next create a catkin package in your catkin workspace
```
cd ~/catkin_ws/src
catkin_create_pkg two_wheel_driver std_msgs rospy
cd two_wheel_driver
mkdir scripts
cd scripts
touch cmd_vel_sub.py
chmod +x cmd_vel_sub.py
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
This should compile fine since theres nothing actually here. But if not hit me up on the scuttle discord! @Monkeyhook (display name Christian Resgreen Group)

Now we need to create the script that subscribes to a topic called cmd_vel. cmd_vel contains data with target velocities for the robot. Many different things can generate a target velocity but in this example the teleop_twist_keyboard will handle that. When you use that program and hit keys on the keyboard is sends a target m/s to linear.x, linear.y, linear.z, angular.x, angular.y, angular.z. It's alot of useless stuff except for 2 of those velocities.
Scuttle only cares about linear.x and angular.z. In our script linear.x is the "drive" velocity and the angular.z is the "rotate" velocity target. This is OPEN LOOP so its not perfect or attempting to match the target. Currently works with up to 1 m/s due to a bug. Anyways, the code for cmd_vel_sub is posted below. Copy and paste into your file with the editor of your choice.
```
#!/usr/bin/env python 
# Import external libraries
import rospy
from geometry_msgs.msg import Twist # source of the joystick messages
import RPi.GPIO as GPIO # Used for gpio pins to control motors
import numpy as np  # use numpy to build the angles array
import time         # for keeping time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
frq = 150                                   # motor driving frequency
# Broadcom (BCM) pin numbering for RasPi is as follows: PHYSICAL:       NAME:
left_chA  = GPIO.PWM(22, frq)     # PIN 11        GPIO22
left_chB  = GPIO.PWM(23, frq)     # PIN 12        GPIO23
right_chA = GPIO.PWM(17, frq)     # PIN 15        GPIO17
right_chB = GPIO.PWM(18, frq)     # PIN 16        GPIO18

# Start PWm at 0 for all motors for safety sake
left_chB.start(0)
left_chA.start(0)
right_chB.start(0)
right_chA.start(0)

def computePWM(speed):              # take an argument in range [-1,1]
    if speed == 0:
        x = np.array([0,0])         # set all PWM to zero
    else:
        x = speed + 1.0             # change the range to [0,2]
        chA = 0.5 * x               # channel A sweeps low to high
        chB = 1 - (0.5 * x)         # channel B sweeps high to low
        x = np.array([chA, chB])    # store values to an array
        x = np.round(x,2)           # round the values
    return(x)

def sendLeft(mySpeed):          # takes at least 0.3 ms
    myPWM = computePWM(mySpeed)
    left_chB.ChangeDutyCycle(myPWM[0] * 100)
    left_chA.ChangeDutyCycle(myPWM[1] * 100)

def sendRight(mySpeed):         # takes at least 0.3 ms
    myPWM = computePWM(mySpeed)
    right_chB.ChangeDutyCycle(myPWM[0] * 100)
    right_chA.ChangeDutyCycle(myPWM[1] * 100)

def varmap(x, in_min, in_max, out_min, out_max):
    return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

def callback(msg):
	rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
	rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
	drive = msg.linear.x
	rotate = msg.angular.z
	maximum = max(abs(drive), abs(rotate))
	total, difference = drive + rotate, drive - rotate
	# set speed according to the quadrant that the values are in
	
	print("drive: ",drive," rotate: ", rotate)
	
	if drive >= 0:
		if rotate >= 0:  # I quadrant
			sendLeft(maximum)
			sendRight(difference)
		else:            # II quadrant
			sendLeft(total)
			sendRight(maximum)
	else:
		if rotate >= 0:  # IV quadrant
			sendLeft(total)
			sendRight(-maximum)
		else:            # III quadrant
			sendLeft(-maximum)
			sendRight(difference)
 
if __name__=="__main__":
	rospy.init_node("cmd_vel_sub")
	r = rospy.Rate(10) # 10hz
	my_subscriber = rospy.Subscriber("/cmd_vel", Twist, callback)
	while not rospy.is_shutdown():
		r.sleep()
```
Save the file and make sure to have three terminals open
1: terminal for roscore
```
roscore
```
2: terminal for teleop_joy_keyboard
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```
![alt text](https://github.com/ansarid/SCUTTLE_ROS/blob/main/src/two_wheel_driver/images/keyboard%20explained.PNG)


Notes: This works with the keys shown. i is forwards, j is left, comma is backwards, l is right, k is stop and the u o n . keys are mixes of the other keys. 
You can adjust your speed but by default this will work with .5 for speed and 1.0 for turn. Theres a guide on screen as well when you run it.

3: terminal for cmd_vel_sub.py run
```
rosrun two_wheel_driver cmd_vel_sub.py
```

Drive around! Return to terminal 2, and the use the keys to test your motors. If all is well your should see correct motion.
