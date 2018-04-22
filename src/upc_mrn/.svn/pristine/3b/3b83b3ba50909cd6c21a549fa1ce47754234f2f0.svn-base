#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

class SubOdom
{
  public:
    SubOdom()
    {
      odom_sub  = n.subscribe("odom", 1, &SubOdom::odom_callback, this);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {    
      double x = odom_msg->pose.pose.position.x;
      double y = odom_msg->pose.pose.position.y;
      double yaw = tf::getYaw(odom_msg->pose.pose.orientation);
      double v = odom_msg->twist.twist.linear.x;
      double w = odom_msg->twist.twist.angular.z;
      ROS_INFO("Read from odom msg: x,y,yaw: %f,%f,%f, v,w: %f,%f", x,y,yaw,v,w);
    }

  private:
    ros::NodeHandle n; 
    ros::Subscriber odom_sub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub_odom");
  SubOdom my_sub_odom;
  ros::spin();
  return 0;
}