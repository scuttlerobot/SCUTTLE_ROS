#!/usr/bin/env python
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import smbus2       # a Python package to communicate over i2c
import numpy as np  # use numpy to build the angles array
import time         # for keeping time

#Parameters
wheeltrack = 0.143
wheelradius = 0.0415
TPR = 360
left_ticks = 0
right_ticks = 0
last_left_ticks = 0
last_right_ticks = 0

x = 0.0
y = 0.0
th = 0.0

vx =  0.0
vy =  0.0
vth =  0.0

rospy.init_node('odom_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10)

bus=smbus2.SMBus(1) # declare the i2c bus object

encL = 0x43         # encoder i2c address for LEFT motor
encR = 0x41         # encoder i2c address for RIGHT motor (this encoder has A1 pin pulled high)
def singleReading(encoderSelection):                                            # return a reading for an encoder in degrees (motor shaft angle)
    try:
        twoByteReading = bus.read_i2c_block_data(encoderSelection, 0xFE, 2)     # request data from registers 0xFE & 0xFF of the encoder. Approx 700 microseconds.
        binaryPosition = (twoByteReading[0] << 6) | twoByteReading[1]           # remove unused bits 6 & 7 from byte 0xFF creating 14 bit value
        degreesPosition = binaryPosition*(float(360)/(2**14))                            # convert to degrees
        degreesAngle = round(degreesPosition,1)                                 # round to nearest 0.1 degrees
    except:
        print("Encoder reading failed.")                                        # indicate a failed reading
        degreesAngle = 0
    return degreesAngle

def readShaftPositions():                                   # read both motor shafts.  approx 0.0023 seconds.
    try:
        rawAngle = singleReading(encL)                      # capture left motor shaft
        angle0 = 360.0 - rawAngle                           # invert the reading for left side only
        angle0 = round(angle0,1)                            # repeat rounding due to math effects
    except:
        print('Warning(I2C): Could not read left encoder')  # indicate which reading failed
        angle0 = 0
    try:
        angle1 = singleReading(encR)                        # capture right motor shaft
    except:
        print('Warning(I2C): Could not read right encoder') # indicate which reading failed
        angle1 = 0
    angles = np.array([angle0,angle1])
    return angles

while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    left_ticks = readShaftPositions()[0]
    right_ticks = readShaftPositions()[1]
    delta_L = left_ticks - last_left_ticks
    delta_R = right_ticks - last_right_ticks
    dl = 2 * pi * wheelradius * delta_L / TPR
    dr = 2 * pi * wheelradius * delta_R / TPR
    dc = (dl + dr) / 2
    dt = (current_time - last_time).to_sec()
    dth = (dr-dl)/wheeltrack

    if dr==dl:
        dx=dr*cos(th)
        dy=dr*sin(th)

    else:
        radius=dc/dth

        iccX=x-radius*sin(th)
        iccY=y+radius*cos(th)

        dx = cos(dth) * (x-iccX) - sin(dth) * (y-iccY) + iccX - x
        dy = sin(dth) * (x-iccX) + cos(dt) * (y-iccY) + iccY - y

    x += dx  
    y += dy 
    th =(th+dth) %  (2 * pi)

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
       (x, y, 0.),
       odom_quat,
       current_time,
       "base_link",
       "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    if dt>0:
       vx=dx/dt
       vy=dy/dt
       vth=dth/dt

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    odom_pub.publish(odom)

    last_left_ticks = left_ticks
    last_right_ticks = right_ticks
    last_time = current_time
    r.sleep()
