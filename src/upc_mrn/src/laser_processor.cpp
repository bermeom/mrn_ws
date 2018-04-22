#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/ColorRGBA.h"
#include <tf/transform_datatypes.h>

class LaserProcessor
{
  public:
    LaserProcessor()
    {
      this->laser_sub  = n.subscribe("scan", 1, &LaserProcessor::laser_callback, this);
      this->odom_sub  = n.subscribe("odom", 1, &LaserProcessor::odom_callback, this);
      this->markers_pub = n.advertise<visualization_msgs::MarkerArray>("markers", 1);

      this->red.r=1.0;
      this->red.a=0.5;
      this->green.g=1.0;
      this->green.a=0.5;
      this->blue.b=1.0;
      this->blue.a=0.5;

      //transform from odom to platform (updated on odom_callback)
      this->pose_x =0.0;
      this->pose_y =0.0;
      this->pose_th=0.0;

      //transform from platform to sensor
      this->static_x =-0.0870;
      this->static_y = 0.0125;
      this->static_z = 0.2972;
      this->static_th= 0.0;
    }

    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
    {
      std::vector<double> x;
      std::vector<double> y;
      int size=laser_msg->ranges.size();
      ROS_INFO("Received scan ranges size=%d",size);
      x.resize(size,0.0);
      y.resize(size,0.0);

      //only take into account 3 values (first, mid, last), for visualization
      std::vector<int> indexes;
      indexes.push_back(0);
      indexes.push_back(size/2);
      indexes.push_back(size-1);

      // //Some examples of using std::vector

      // //Resize a vector
      // std::vector<int> v;
      // int my_size=2;
      // ROS_INFO("v.size()=%lu", v.size());
      // v.resize(my_size);
      // ROS_INFO("v.size()=%lu", v.size());
      // ROS_INFO("--");

      // //Add element by element
      // std::vector<double> vv;
      // double my_number=1.23;
      // double other_number=4.56;
      // ROS_INFO("vv.size()=%lu", vv.size());
      // vv.push_back(my_number);
      // ROS_INFO("vv.size()=%lu", vv.size());
      // vv.push_back(other_number);
      // ROS_INFO("vv.size()=%lu", vv.size());
      // ROS_INFO("--");

      // //Init vector with size and value
      // int other_size=3;
      // double value=7.89;
      // std::vector<double> vvv(other_size,value);
      // //Loop through a vector
      // for(unsigned int i=0; i<vvv.size(); i++)
      // {
      //  ROS_INFO("vvv[%d]=%f",i, vvv[i]);
      // }
      // ROS_INFO("--");

      // //

      //TODO 1 START
      double th;
      for (int i=0;i<size;i++){
          th=laser_msg->angle_min+i*laser_msg->angle_increment;
          x[i]=(laser_msg->ranges[i]*cos(th));
          y[i]=(laser_msg->ranges[i]*sin(th));
      }
      //TODO 1 END

      //OUTPUT1
      ROS_INFO("Cartesian");
      for(unsigned int i=0; i<indexes.size(); i++){
        ROS_INFO("for index=%d, x,y=%f,%f",indexes[i], x[indexes[i]],y[indexes[i]]);
      }
      this->publish_markers(x,y,indexes,"camera_depth_frame","Cartesian",0.0,red,laser_msg->header.stamp);

      //TODO 2 START
      double xcami, ycami;
      double xstai, ystai;
      double xodoi, yodoi;

      for (int i=0;i<size;i++){
        xcami = x[i];
        ycami = y[i];
        xstai = xcami + this->static_x;
        ystai = ycami + this->static_y;
        xodoi = cos(this->pose_th)*xstai - sin(this->pose_th)*ystai + this->pose_x;
        yodoi = sin(this->pose_th)*xstai + cos(this->pose_th)*ystai + this->pose_y;
        x[i] = xodoi;
        y[i] = yodoi;
      }
      //TODO 2 END
      //OUTPUT2
      ROS_INFO("World");
      for(unsigned int i=0; i<indexes.size(); i++){
        ROS_INFO("for index=%d, x,y=%f,%f",indexes[i], x[indexes[i]],y[indexes[i]]);
      }
      this->publish_markers(x,y,indexes,"odom","world",this->static_z,blue,laser_msg->header.stamp);

      ROS_INFO("---");
    }

    void publish_markers(std::vector<double> x, std::vector<double> y, std::vector<int> indexes, std::string frame_id, std::string name_space,double z, std_msgs::ColorRGBA color, ros::Time t)
    {

      //this->markers_msg.markers.resize(indexes.size());
      this->markers_msg.markers.clear();
      for(unsigned int i=0; i<indexes.size(); i++)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp    = t;
        marker.ns              = name_space;
        marker.id              = i;
        //marker.id              = this->markers_msg.markers.size();
        marker.type            = visualization_msgs::Marker::SPHERE;
        marker.action          = visualization_msgs::Marker::ADD;

        double radius=0.1;
        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.scale.z = radius;

        marker.pose.position.x = x[indexes[i]];
        marker.pose.position.y = y[indexes[i]];
        marker.pose.position.z = z;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        //marker.color.r = 1.0;
        //marker.color.g = 0.0;
        //marker.color.b = 0.0;
        //marker.color.a = 0.5;
        marker.color=color;
        marker.lifetime = ros::Duration(0.0f);

        //only publish if not NaNs
        if(x[indexes[i]]==x[indexes[i]] && y[indexes[i]] == y[indexes[i]])
        {
          this->markers_msg.markers.push_back(marker);
        }
      }
      this->markers_pub.publish(this->markers_msg);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
      this->pose_x = odom_msg->pose.pose.position.x;
      this->pose_y = odom_msg->pose.pose.position.y;
      this->pose_th = tf::getYaw(odom_msg->pose.pose.orientation);
    }

    void loop()
    {
      //empty
    }

  private:
    ros::NodeHandle n;
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;
    ros::Publisher markers_pub;
    visualization_msgs::MarkerArray markers_msg;
    std_msgs::ColorRGBA red;
    std_msgs::ColorRGBA green;
    std_msgs::ColorRGBA blue;
    double pose_x, pose_y, pose_th;
    double static_x, static_y, static_z, static_th;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_processor");
  LaserProcessor my_laser_processor;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    my_laser_processor.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}