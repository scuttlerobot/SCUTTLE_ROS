#!/usr/bin/python3

import tf
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header
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

        while not rospy.is_shutdown():

            current_time = rospy.Time.now()

            y, x = scuttle.getGlobalPosition()
            vx, vth = scuttle.getMotion()
            th = scuttle.getHeading()

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            odom_broadcaster.sendTransform(
            (x, -y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
            )

            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            odom.pose.pose = Pose(Point(x, -y, 0.), Quaternion(*odom_quat))

            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(0, vx, 0), Vector3(0, 0, 0))

            odom_pub.publish(odom)

            myJointStatePublisher = rospy.Publisher('joint_states', JointState, queue_size=10)
            # pub = rospy.Publisher('joint_states', JointState, queue_size=10)
            # rospy.init_node('joint_state_publisher')
            jointState = JointState()

            jointState.header = Header()

            jointState.header.stamp = rospy.Time.now()

            jointState.name = ['l_wheel_joint',
                               'r_wheel_joint',
                               'r_caster_swivel_joint',
                               'l_caster_swivel_joint',
                               'r_caster_wheel_joint',
                               'l_caster_wheel_joint'
                              ]

            jointState.position = [scuttle.leftWheel.encoder.position * ((2 * np.pi) / scuttle.leftWheel.encoder.resolution),
                                   scuttle.leftWheel.encoder.position * ((2 * np.pi) / scuttle.leftWheel.encoder.resolution),
                                   0,
                                   0,
                                   0,
                                   0
                                  ]

            jointState.velocity = []
            jointState.effort = []
            myJointStatePublisher.publish(jointState)

            r.sleep()

    except rospy.ROSInterruptException:
        pass

    except KeyboardInterrupt:
        print('Stopping...')
        # pass

    finally:
        scuttle.stop()