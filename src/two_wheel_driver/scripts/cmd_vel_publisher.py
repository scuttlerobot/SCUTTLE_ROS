#!/usr/bin/env python 
 
import rospy
from geometry_msgs.msg import Twist
 
if __name__=="__main__":
    rospy.init_node("my_publisher_node")
    my_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    my_velocity = Twist()
    my_velocity.linear.x = 0.5
    my_velocity.angular.z = 0.5
 
    while not rospy.is_shutdown():
        my_publisher.publish(my_velocity)
        rate.sleep()
