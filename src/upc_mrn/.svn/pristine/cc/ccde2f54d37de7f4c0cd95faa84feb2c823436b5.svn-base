#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf

def odom_callback(odom_msg):
    x=odom_msg.pose.pose.position.x
    y=odom_msg.pose.pose.position.y   
    yaw = tf.transformations.euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])[2]
    v=odom_msg.twist.twist.linear.x
    w=odom_msg.twist.twist.angular.z
    rospy.loginfo("Read from odom msg: x,y,yaw: %f,%f,%f, v,w: %f,%f", x,y,yaw,v,w)

def sub_odom():
    rospy.init_node('sub_odom', anonymous=True)
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        sub_odom()
    except rospy.ROSInterruptException:
        pass