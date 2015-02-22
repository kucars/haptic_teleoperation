#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud{
public:

  ros::NodeHandle n_;

  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LaserScanToPointCloud(ros::NodeHandle n): 
    n_(n),
   // laser_sub_(n_, "base_scan", 10),
    laser_sub_(n_, "/scan", 100),
    laser_notifier_(laser_sub_,listener_, "laser", 1)
  	{
      std::cout << "Object created" << std::endl ;

   	 	laser_notifier_.registerCallback(
        boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));// 0.01
        std::cout << "before pub " << std::endl ;
        scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/cloud",1000);
        std::cout << "after pub " << std::endl ;

  	}

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
      std::cout << "scanCallback " << std::endl ;

    sensor_msgs::PointCloud cloud;
    try
    {
        std::cout << "try " << std::endl ;

        projector_.transformLaserScanToPointCloud(
          "base_link",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << "catch 1 " << std::endl ;

        std::cout << e.what();
        std::cout << "catch " << std::endl ;

        return;
    }
    std::cout << "cloud" << cloud << std::endl ;
    scan_pub_.publish(cloud);

  }
};

int main(int argc, char** argv)
{
    std::cout << "Int Main laser 2 point cloud" << std::endl ;

  
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");
  double freq;
  n_priv.param<double>("frequency", freq, 50.0);
  ros::Rate loop_rate(freq);

  LaserScanToPointCloud lstopc(n);

  std::cout << "Object created" << std::endl ;

  while(ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
