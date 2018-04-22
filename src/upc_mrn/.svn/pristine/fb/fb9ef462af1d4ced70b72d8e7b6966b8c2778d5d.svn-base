#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_datatypes.h>

class OdomNew
{
  public:
    OdomNew()
    {
      this->joint_state_sub  = n.subscribe("joint_states", 1, &OdomNew::joint_state_callback, this);
      this->received_joint_state = false;
      
      this->odom_sub  = n.subscribe("odom", 1, &OdomNew::odom_callback, this);
      this->received_odom = false;
      this->get_x_y_th    = true;
      
      this->odom_pub = n.advertise<nav_msgs::Odometry>("odom_new", 1);
      this->x=0.0;
      this->y=0.0;
      this->th=0.0;
      
      this->radius=0.035;
      this->wheel_distance=0.23;
      this->last_time = ros::Time::now();
    }

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      this->joint_state_msg = *msg;
      this->received_joint_state=true;
    }
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      if(!this->received_odom)
      {
        ROS_INFO("Received input odom");
        this->input_odom_msg = *msg;
        this->received_odom=true;
      }
    }

    void compute_and_publish_odom()
    {
      //ROS_INFO("compute_and_publish_odom");
      double left_pos, right_pos, left_vel, right_vel;
      if(this->joint_state_msg.position.size()>=2 && this->joint_state_msg.velocity.size()>=2)
      {
        left_pos  = this->joint_state_msg.position[0]; //CAUTION name field should be checked, for now, assuming 0=left, 1=right
        right_pos = this->joint_state_msg.position[1];
        left_vel  = this->joint_state_msg.velocity[0];
        right_vel = this->joint_state_msg.velocity[1];
      }
      else
        ROS_ERROR("Wrong input joint_state size");

      ros::Time time = this->joint_state_msg.header.stamp;
      //ROS_INFO("Received left_pos,right_pos: %f,%f, left_vel,right_vel: %f,%f", left_pos, right_pos, left_vel, right_vel);
      double dt = (time - this->last_time).toSec();
      //ROS_INFO("dt: %fs",dt);
      double vx=0.0;
      double vth=0.0;
      
      //////// TODO //////
      //vx  = 
      //vth = 
      //this->x  = 
      //this->y  = 
      //this->th = 
      //////// TODO //////
      
      this->odom_msg.header.stamp = time;
      this->odom_msg.header.frame_id = "odom";
      this->odom_msg.pose.pose.position.x = this->x;
      this->odom_msg.pose.pose.position.y = this->y;
      this->odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->th);
      //this->odom_msg.pose.covariance=
      this->odom_msg.child_frame_id = "base_footprint_new";
      this->odom_msg.twist.twist.linear.x = vx;
      this->odom_msg.twist.twist.angular.z = vth;
      //this->odom_msg.twist.covariance=
      
      this->odom_pub.publish(this->odom_msg);
      this->last_time=time;
    }
    
    void loop()
    {
      //ROS_INFO("loop");
      if(received_odom && get_x_y_th)
      {
        this->x = this->input_odom_msg.pose.pose.position.x;
        this->y = this->input_odom_msg.pose.pose.position.y;
        this->th = tf::getYaw(this->input_odom_msg.pose.pose.orientation);
        this->last_time = this->input_odom_msg.header.stamp;
        ROS_INFO("Initialization: getting from input odom x,y,th=%f,%f,%f", this->x,this->y,this->th);
        this->get_x_y_th=false;
      }
      
      if(this->received_joint_state)
        this->compute_and_publish_odom();
    }

  private:
    ros::NodeHandle n; 
    ros::Subscriber odom_sub;
    nav_msgs::Odometry input_odom_msg;
    bool received_odom;
    bool get_x_y_th;
    ros::Subscriber joint_state_sub;
    sensor_msgs::JointState joint_state_msg;
    bool received_joint_state;
    ros::Publisher  odom_pub;
    nav_msgs::Odometry odom_msg;
    double x,y,th;
    double radius, wheel_distance;
    ros::Time last_time;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_new");
  OdomNew my_odom_new;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    my_odom_new.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
