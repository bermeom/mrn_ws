#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def pub_twist():
    rospy.init_node('pub_twist', anonymous=True)
    twist_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    twist_msg = Twist()
    while not rospy.is_shutdown():
        twist_msg.linear.x  = 0.25
        twist_msg.angular.z = 0.5
        twist_pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        pub_twist()
    except rospy.ROSInterruptException:
        pass