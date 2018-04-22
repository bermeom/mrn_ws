#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
from math import cos, sin

class odom_new():

    def __init__(self):
        rospy.init_node('odom', anonymous=True)
        self.jointstates_sub = rospy.Subscriber("joint_states", JointState, self.joint_state_callback)
        self.joint_state_msg = JointState()
        self.received_joint_state = False
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.input_odom_msg = Odometry()
        self.received_odom = False
        self.get_x_y_th = True
        self.odom_pub = rospy.Publisher('odom_new', Odometry, queue_size=1)
        self.odom_msg = Odometry()
        self.x = 0
        self.y = 0
        self.th= 0
        self.rate = rospy.Rate(10)
        self.radius=0.035
        self.wheel_distance=0.23
        self.last_time = rospy.Time()

    def joint_state_callback(self,msg):
        self.joint_state_msg = msg
        self.received_joint_state = True
        
    def odom_callback(self,msg):
        if not self.received_odom:
          rospy.loginfo("Received input odom")
          self.input_odom_msg = msg
          self.received_odom = True
        
    def compute_and_publish_odom(self):
        #rospy.loginfo("compute_and_publish_odom")
        size=2
        if(len(self.joint_state_msg.position) >= size and len(self.joint_state_msg.velocity)>= size):
          left_pos  = self.joint_state_msg.position[0] #CAUTION name field should be checked, for now, assuming 0=left, 1=right
          right_pos = self.joint_state_msg.position[1]
          left_vel  = self.joint_state_msg.velocity[0]
          right_vel = self.joint_state_msg.velocity[1]
        else:
          rospy.logerr("Wrong input joint_state size %d<%d",len(self.joint_state_msg.position), size)
        time = self.joint_state_msg.header.stamp
        #rospy.loginfo("Received left_pos,right_pos: %f, %f, left_vel,right_vel: %f,%f", left_pos, right_pos, left_vel, right_vel)
        dt = (time - self.last_time).to_sec()
        #rospy.loginfo("dt: %fs",dt);
        vx=0
        vth=0

        #### TO DO ###
        #vx = 
        #vth = 
        #self.x  =
        #self.y  =
        #self.th =
        #### TO DO ###

        self.odom_msg.header.stamp = time
        self.odom_msg.header.frame_id = "odom"
        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_msg.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*q))
        #self.odom_msg.pose.covariance=
        self.odom_msg.child_frame_id = "base_footprint_new"
        self.odom_msg.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))
        #self.odom_msg.twist.covariance=
        self.odom_pub.publish(self.odom_msg)
        self.last_time=time
        
    def loop(self):
        #rospy.loginfo("loop")
        if self.received_odom and self.get_x_y_th:
          self.x = self.input_odom_msg.pose.pose.position.x
          self.y = self.input_odom_msg.pose.pose.position.y
          self.th = tf.transformations.euler_from_quaternion([self.input_odom_msg.pose.pose.orientation.x, self.input_odom_msg.pose.pose.orientation.y, self.input_odom_msg.pose.pose.orientation.z, self.input_odom_msg.pose.pose.orientation.w])[2]
          self.last_time = self.input_odom_msg.header.stamp
          rospy.loginfo("Initialization: getting from input odom x,y,th=%f,%f,%f",self.x,self.y,self.th)
          self.get_x_y_th = False
          
        if self.received_joint_state:
            self.compute_and_publish_odom()
      

if __name__ == '__main__':
    try:
        #Testing our function
        n = odom_new()
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            n.loop()
            rate.sleep()

    except rospy.ROSInterruptException: pass