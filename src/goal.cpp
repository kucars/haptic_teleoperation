#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "laser_geometry/laser_geometry.h"
#include <Eigen/Eigen>
#include "std_msgs/String.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;

  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud>("goal", 50);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
   std::vector<std::string> list;
   list.clear();
   list.push_back("/goal");


  while (node.ok()){
    tf::StampedTransform transform;
    try{
      sensor_msgs::PointCloud cloud;
      cloud.header.stamp = ros::Time::now();
      cloud.header.frame_id = "/base_link";

       for ( int i=0 ; i < list.size() ; i++)// FOR EACH OBSTACLE
      {
       listener.waitForTransform("/base_link",list[i], ros::Time(0), ros::Duration(10.0) );
       listener.lookupTransform("/base_link",list[i], ros::Time(0), transform);

       geometry_msgs::Point32 point;
       point.x =transform.getOrigin().x();
       point.y =transform.getOrigin().y();
       point.z =transform.getOrigin().z();
       cloud.points.push_back(point);


      //we'll also add an intensity channel to the cloud
       sensor_msgs::ChannelFloat32 intensity_channel;
       intensity_channel.name="intensity";
       intensity_channel.values.push_back(1.0);
       cloud.channels.push_back(intensity_channel);
      }
      cloud_pub.publish(cloud);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    rate.sleep();
  }
  return 0;
};
