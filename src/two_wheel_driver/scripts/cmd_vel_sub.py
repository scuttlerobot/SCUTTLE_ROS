#!/usr/bin/env python 
 
import rospy
from geometry_msgs.msg import Twist
# Import external libraries
import RPi.GPIO as GPIO
import numpy as np  # use numpy to build the angles array
import time         # for keeping time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
frq = 150                                   # motor driving frequency
# Broadcom (BCM) pin numbering for RasPi is as follows: PHYSICAL:       NAME:
left_chA  = GPIO.PWM(22, frq)     # PIN 11        GPIO17
left_chB  = GPIO.PWM(23, frq)     # PIN 12        GPIO18
right_chA = GPIO.PWM(17, frq)     # PIN 15        GPIO22
right_chB = GPIO.PWM(18, frq)     # PIN 16        GPIO23

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
