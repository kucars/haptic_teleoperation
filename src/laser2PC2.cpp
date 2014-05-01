#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloudTwo{

public:
public:
      LaserScanToPointCloudTwo();
      void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
   private:
      ros::NodeHandle node_;
      laser_geometry::LaserProjection projector_;
      tf::TransformListener tfListener_;

      ros::Publisher point_cloud_publisher_;
      ros::Subscriber scan_sub_;
};

LaserScanToPointCloudTwo::LaserScanToPointCloudTwo(){
      scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &LaserScanToPointCloudTwo::scanCallback, this);
      point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud_in", 100, false);
      tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}


void LaserScanToPointCloudTwo::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("/base_link", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}

int main(int argc, char** argv)
{
  
    ros::init(argc, argv, "LaserScanToPointCloudTwo");

      LaserScanToPointCloudTwo filter;

      ros::spin();

      return 0;
}
