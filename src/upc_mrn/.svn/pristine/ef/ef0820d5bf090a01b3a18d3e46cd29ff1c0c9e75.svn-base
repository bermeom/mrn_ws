#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class PubTwist
{
  public:
    PubTwist()
    {
      twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
    }

    void publish_twist()
    {
      twist_msg.linear.x  = 0.25;
      twist_msg.angular.z = 0.5;
      twist_pub.publish(twist_msg);
    }

  private:
    ros::NodeHandle n; 
    ros::Publisher  twist_pub;
    geometry_msgs::Twist twist_msg;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_twist");
  PubTwist my_pub_twist;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    my_pub_twist.publish_twist();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}