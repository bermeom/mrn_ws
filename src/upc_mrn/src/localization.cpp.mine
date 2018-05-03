#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/ColorRGBA.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


struct Cluster
{
  double x;
  double y;
  double width;
  int start_index;
 
  bool operator < (const Cluster& c) const
        {
        return (width > c.width);
        }
};

struct Landmark
{
  double x;
  double y;
  double radius;
};

struct Pose
{
  double x;
  double y;
  double th;
};

class Localization
{
  public:
    Localization()
    {
      this->laser_sub  = n.subscribe("laserscan", 1, &Localization::laser_callback, this);
      this->markers_pub = n.advertise<visualization_msgs::MarkerArray>("markers", 1);
      this->pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose",1);
      
      this->red.r=1.0;
      this->red.a=0.5;
      this->green.g=1.0;
      this->green.a=0.5;
      this->blue.b=1.0;
      this->blue.a=0.5;
      
      //transform from platform to sensor
      this->static_x = 0.00; //[m]
      this->static_y = 0.00; //[m]
      this->static_z = 0.44; //[m]//ignore. Used for displaying.
      this->static_th= 0.0; //[radians]
      
      this->landmarks.resize(3);
      //Coordinates associated to localization.world cylinders positions and sizes
      this->landmarks[0].x    = -2.0;
      this->landmarks[0].y    =  2.0;
      this->landmarks[0].radius=  0.5;
      this->landmarks[1].x    =  0.0;
      this->landmarks[1].y    = -1.0;
      this->landmarks[1].radius=  0.3;
      this->landmarks[2].x    =  1.0;
      this->landmarks[2].y    =  1.0;
      this->landmarks[2].radius=  0.15;

      this->received_scan=false;
      this->rate=2; //HZ
    }
  
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      this->scan_msg = *msg;
      this->received_scan=true;
    }

    std::vector<Cluster> getClusters()
    {
      std::vector<Cluster> clusters;

      int size=this->scan_msg.ranges.size();
      double th;
      std::vector<double> x;
      std::vector<double> y;
      
      //TODO 1 (1.1 to 1.5)
      //Fill clusters.
      
      //Polar to Cartesian (x,y)
      for(unsigned int i=0; i<size; i++)
      {
        if(!isinf(this->scan_msg.ranges[i]))
        {
          //TODO 1.1 BEGIN
          //Polar to Cartesian (x,y). Same as in session 2
          //Add element by element
          th = this->scan_msg.angle_min+i*this->scan_msg.angle_increment;
          x.push_back(this->scan_msg.ranges[i]*cos(th));
          y.push_back(this->scan_msg.ranges[i]*sin(th));
          // ROS_INFO("for index=%d, x,y=%f,%f",i, x.back(),y.back());
      
          //TODO 1.1 END
        }
      }

      //Group points and fill clusters (x,y,width)
      
      //Add 0,0 point to close last cluster
      x.push_back(0.0);
      y.push_back(0.0);

      size=x.size();
      double jumpdist=0.2;
      double dist=0.0;
        
      //First cluster, starts on first point (0)
      clusters.resize(1);
      clusters[0].start_index=0;
      clusters[0].x=x[0];
      clusters[0].y=y[0];
      clusters[0].width=1e-9;
      
      int p1,p2,p3;
      Eigen::MatrixXd A(2,2);
      Eigen::MatrixXd B(2,1);
      Eigen::MatrixXd Xc(2,1);

      for (unsigned int i=1; i<size; i++) {
        //TODO 1.2 BEGIN
        //Euclidean distance between current point and previous point
        dist = sqrt((x[i]-x[i-1])*(x[i]-x[i-1])+(y[i]-y[i-1])*(y[i]-y[i-1]));
        //TODO 1.2 END
        if (dist>jumpdist) {

          if (i-clusters.back().start_index>=3){
              p1 = clusters.back().start_index;
              p3 = i-1;
              p2 = (p3-p1)/2+p1;
              clusters.back().x = (x[p1]*x[p1]*y[p2] - x[p1]*x[p1]*y[p3] - x[p2]*x[p2]*y[p1] + x[p2]*x[p2]*y[p3] + x[p3]*x[p3]*y[p1] - x[p3]*x[p3]*y[p2] + y[p1]*y[p1]*y[p2] - y[p1]*y[p1]*y[p3] - y[p1]*y[p2]*y[p2] + y[p1]*y[p3]*y[p3] + y[p2]*y[p2]*y[p3] - y[p2]*y[p3]*y[p3])/(2*(x[p1]*y[p2] - x[p2]*y[p1] - x[p1]*y[p3] + x[p3]*y[p1] + x[p2]*y[p3] - x[p3]*y[p2]));
              //                  (x1^2*y2           - x1^2*y3           - x2^2*y1           + x2^2*y3           + x3^2*y1           - x3^2*y2           + y1^2*y2           - y1^2*y3           - y1*y2^2           + y1*y3^2           + y2^2*y3           - y2*y3^2          )/(2*(x1*y2       - x2*y1       - x1*y3       + x3*y1       + x2*y3       - x3*y2      ))
              clusters.back().y = (- x[p1]*x[p1]*x[p2] + x[p1]*x[p1]*x[p3] + x[p1]*x[p2]*x[p2] - x[p1]*x[p3]*x[p3] + x[p1]*y[p2]*y[p2] - x[p1]*y[p3]*y[p3] - x[p2]*x[p2]*x[p3] + x[p2]*x[p3]*x[p3] - x[p2]*y[p1]*y[p1] + x[p2]*y[p3]*y[p3] + x[p3]*y[p1]*y[p1] - x[p3]*y[p2]*y[p2])/(2*(x[p1]*y[p2] - x[p2]*y[p1] - x[p1]*y[p3] + x[p3]*y[p1] + x[p2]*y[p3] - x[p3]*y[p2]));
              //                  (- x1^2*x2           + x1^2*x3           + x1*x2^2           - x1*x3^2           + x1*y2^2           - x1*y3^2           - x2^2*x3           + x2*x3^2           - x2*y1^2           + x2*y3^2           + x3*y1^2           - x3*y2^2          )/(2*(x1*y2       - x2*y1       - x1*y3       + x3*y1       + x2*y3       - x3*y2      ))
              clusters.back().width =  sqrt((x[p1]-clusters.back().x)*(x[p1]-clusters.back().x)+(y[p1]-clusters.back().y)*(y[p1]-clusters.back().y));
              // ROS_INFO("->>> X=[%f;%f;%f],Y=[%f;%f;%f]  %% xc=%f, yc=%f, r=%f",x[p1],x[p2],x[p3],y[p1],y[p2],y[p3],clusters.back().x,clusters.back().y,clusters.back().width);          
          }else{
              switch(i-clusters[i].start_index){
                    case(1):clusters.back().width=1e-9;break;
                    case(2):clusters.back().width=1e-6;break;
              }
          }
          //----------------------------------------------------------
          Cluster c;
          c.start_index=i;
          c.x=x[i];
          c.y=y[i];
          c.width=1e-9;
          clusters.push_back(c);
        }
      }
      //TODO 1.5 BEGIN
      // Clean clusters down to having 3 and/or match clusters with the 3 landmarks
      
      // A cluster on the back of the robot can be divided by the scan begin/end. 
      // You can try to detect that and join them in one
      
      // Match clusters with our 3 landmarks.
      // You can reduce the number of clusters to 3, before matching them with landmarks
      // Or you can directly match them (cluster.width <--> landmark.radius)
      

      ////For example, this simply sorts clusters by width (max...min) and then deletes the last(small) ones until having only 3
      std::sort(clusters.begin(), clusters.end());
      while (clusters.size()>3)
            clusters.pop_back();
      
      //TODO 1.5 END
      for(unsigned int i=0; i<clusters.size(); i++)
        ROS_INFO("cluster[%d] x,y,width=%f, %f, %f,%f", i, clusters[i].x, clusters[i].y, clusters[i].width,this->pose.th);
      return clusters;
    }

    Pose compute_pose()
    {
      Pose p;
      double scale;
      
      if(this->clusters.size() != this->landmarks.size() )
        ROS_WARN("Warning, your clusters number (%lu) is different from the landmarks number (%lu)",this->clusters.size(), this->landmarks.size());

      //TODO 2 (2.1 to 2.3)
      
      //Some matrix and vector examples of usage (using Eigen library):
      // std::cout << "Eigen examples START" << std::endl;
      // int rows = 2;
      // int cols = 4;
      // Eigen::MatrixXd M(rows,cols);
      // Eigen::VectorXd V(cols);
      // V << 2.0,4.4,6.33,8.21;
      // M.row(0) = V;
      // M.row(1) << 2, 20, 200, 2000;
      // std::cout << "M: " << std::endl;
      // std::cout <<  M    << std::endl;
      // std::cout << "M': " << std::endl;
      // std::cout <<  M.transpose()    << std::endl;
      
      // Eigen::MatrixXd N(2,2);
      // N << 1, 2, 3, 4;
      // std::cout << "N: " << std::endl;
      // std::cout <<  N    << std::endl;
      // std::cout << "N^-1': " << std::endl;
      // std::cout <<  N.inverse() << std::endl;
      // std::cout << "Eigen examples END" << std::endl;
      
      //Create A and B matrix (landmarks)
      Eigen::MatrixXd A(this->clusters.size()*2,4);// rows,cols
      Eigen::MatrixXd AA(this->clusters.size()*2,4);// rows,cols
      Eigen::MatrixXd B(this->clusters.size()*2,1);// rows,cols
      unsigned int k;
      for (unsigned int i = 0;i<this->clusters.size();i++){
        k = this->clusters[i].start_index;     
        //Fill A and B matrix (landmarks)
        A.row(2*i)   << this->landmarks[i].x, this->landmarks[i].y,1,0;
        A.row(2*i+1) << this->landmarks[i].y,-this->landmarks[i].x,0,1;
        B.row(2*i)   << this->clusters[i].x;
        B.row(2*i+1) << this->clusters[i].y;
      }
      std::cout << "A: \n " << A << std::endl;
      std::cout << "B: \n" << B << std::endl;
      
      Eigen::VectorXd X = Eigen::VectorXd::Zero(4);
      //TODO 2.3: BEGIN
      //Solve the system for X
      
      // AA = (A.transpose()*A);
      X = (A.transpose()*A).inverse()*A.transpose()*B;
      //TODO 2.3 END
      //This prints X on screen
      std::cout << "X: \n" << X << std::endl;

      p.x = X.coeff(2, 0);
      p.y = X.coeff(3, 0);
      p.th= atan2(-X.coeff(1, 0) , X.coeff(0, 0) );
      scale = sqrt(X.coeff(0, 0)*X.coeff(0, 0)+X.coeff(1, 0)*X.coeff(1, 0));

      ROS_INFO("pose.x,pose.y= %f, %f.   pose.th, scale= %f, %f",p.x, p.y, p.th, scale);
      return p;
    }

    void loop()
    {
      if(this->received_scan)
      {
        this->received_scan=false;

        this->clusters = this->getClusters();
        this->publish_markers(this->clusters,"laser_link","cluster_pos",-this->static_z,this->red,this->scan_msg.header.stamp);

        this->pose = this->compute_pose();
        this->publish_pose(this->pose.x, this->pose.y, this->pose.th, "laser_link", -this->static_z, this->red, this->scan_msg.header.stamp );

        ROS_INFO("---");
      }
    }

    void publish_pose(double x, double y, double th, std::string frame_id, double z, std_msgs::ColorRGBA color, ros::Time t)
    {
      this->pose_msg.header.stamp = t;
      this->pose_msg.header.frame_id = frame_id;
      this->pose_msg.pose.position.x = x;
      this->pose_msg.pose.position.y = y;
      this->pose_msg.pose.position.z = z;
      this->pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(th);
      this->pose_pub.publish(this->pose_msg);
    }

    void publish_markers(std::vector<Cluster> c, std::string frame_id, std::string name_space,double z, std_msgs::ColorRGBA color, ros::Time t)
    {
      this->markers_msg.markers.clear();
      for(unsigned int i=0; i<c.size(); i++)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp     = t;
        marker.ns               = name_space;
        marker.id               = i;
        //marker.id             = this->markers_msg.markers.size();
        marker.type             = visualization_msgs::Marker::CYLINDER;
        marker.action           = visualization_msgs::Marker::ADD;

        double radius=c[i].width;
        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.scale.z = 1.0;

        marker.pose.position.x = c[i].x;
        marker.pose.position.y = c[i].y;
        marker.pose.position.z = z + marker.scale.z/2.0 ;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        //marker.color.r = 1.0;
        //marker.color.g = 0.0;
        //marker.color.b = 0.0;
        //marker.color.a = 0.5;
        marker.color=color;
        marker.lifetime = ros::Duration(1.0/this->rate);

        //only publish if not NaNs
        if(c[i].x==c[i].x && c[i].y == c[i].y)
        {
          this->markers_msg.markers.push_back(marker);
        }
      }
      this->markers_pub.publish(this->markers_msg);
    }

    double getRate()
    {
      return this->rate;
    }

  private:
    ros::NodeHandle n;
    double rate;
    ros::Subscriber laser_sub;
    sensor_msgs::LaserScan scan_msg;
    bool received_scan;
    ros::Publisher markers_pub;
    visualization_msgs::MarkerArray markers_msg;
    ros::Publisher pose_pub;
    geometry_msgs::PoseStamped pose_msg;
    std_msgs::ColorRGBA red;
    std_msgs::ColorRGBA green;
    std_msgs::ColorRGBA blue;
    double static_x, static_y, static_z, static_th;
    std::vector<Landmark> landmarks;
    std::vector<Cluster> clusters;
    Pose pose;
    tf::TransformListener tf_listener;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization");
  Localization my_localization;
  ros::Rate loop_rate(my_localization.getRate());
  while (ros::ok())
  {
    my_localization.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


