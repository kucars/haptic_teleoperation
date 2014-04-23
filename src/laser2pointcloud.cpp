#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
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
    laser_sub_(n_, "laserscan", 1),
    laser_notifier_(laser_sub_,listener_, "/Pioneer3AT/base_link", 1)
  	{
   	 	laser_notifier_.registerCallback(
    	 	boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));// 0.01
        scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("cloud",1);
  	}

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          "/Pioneer3AT/base_link",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
    

    scan_pub_.publish(cloud);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");
  double freq;
  n_priv.param<double>("frequency", freq, 10.0);
  ros::Rate loop_rate(freq);

  LaserScanToPointCloud lstopc(n);
  
  while(ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
