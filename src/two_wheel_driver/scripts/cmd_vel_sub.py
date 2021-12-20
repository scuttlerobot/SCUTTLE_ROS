#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from scuttlepy import SCUTTLE

def callback(msg):

    rospy.loginfo("Linear Velocity: "+str(round(msg.linear.x, 3))+" \tAngular Velocity: "+str(round(msg.angular.z, 3)))
    scuttle.setMotion([msg.linear.x, msg.angular.z])

if __name__=="__main__":
    rospy.init_node("cmd_vel_sub")
    r = rospy.Rate(50) # 50hz
    my_subscriber = rospy.Subscriber("/cmd_vel", Twist, callback)
    scuttle = SCUTTLE(openLoop=True)

    try:

        while not rospy.is_shutdown():

            r.sleep()

    except KeyboardInterrupt:

        pass

    finally:

        scuttle.stop()
