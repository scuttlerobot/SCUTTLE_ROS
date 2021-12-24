#!/usr/bin/python3

import tf
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from scuttlepy import SCUTTLE

def commandVelocity(msg):

    # rospy.loginfo("Linear Velocity: "+str(round(msg.linear.x, 3))+" \tAngular Velocity: "+str(round(msg.angular.z, 3)))
    scuttle.setMotion([msg.linear.x, msg.angular.z])

if __name__=="__main__":

    scuttle = SCUTTLE(openLoop=True)

    rospy.init_node("scuttle_controller")
    r = rospy.Rate(50) # 50hz

    my_subscriber = rospy.Subscriber("/cmd_vel", Twist, commandVelocity)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    try:

        x = 0
        y = 0

        while not rospy.is_shutdown():

            current_time = rospy.Time.now()

            y, x = scuttle.getGlobalPosition()
            y *= -1

            vx, vth = scuttle.getMotion()
            th = scuttle.getHeading()

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
            )

            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(0, vx, 0), Vector3(0, 0, 0))

            odom_pub.publish(odom)

            r.sleep()

    except KeyboardInterrupt:

        pass

    finally:

        scuttle.stop()
