#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "std_msgs/String.h"
#include <sstream>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class ModelState{
public: 
	ros::NodeHandle n_, n_priv_;

  	ModelState(ros::NodeHandle & n) : n_(n), n_priv_("~")
  	{
   	 	n_priv_.param<std::string>("model_name", model_name , "quadrotor");
        nav_msgs_sub = n_.subscribe("/pose",1, &ModelState::positionCallback, this);
   		model_state_pub = n_.advertise<gazebo_msgs::ModelState>("/set_model_state", 1);
  	};


private:
  ros::Publisher model_state_pub ; 
  ros::Subscriber nav_msgs_sub;
  std::string model_name;
  void positionCallback (const nav_msgs::Odometry::ConstPtr& msg )
  {
	std::cout << "In the call function" << std::endl ; 
	//gazebo_msgs::ModelState model; 
	// filling the msgs 	
	geometry_msgs::Pose start_pose;
        start_pose.position.x = msg->pose.pose.position.x;
        start_pose.position.y = msg->pose.pose.position.y;
        start_pose.position.z = msg->pose.pose.position.z;
        start_pose.orientation.x = msg->pose.pose.orientation.x;
        start_pose.orientation.y = msg->pose.pose.orientation.y;
        start_pose.orientation.z = msg->pose.pose.orientation.z;
        start_pose.orientation.w = msg->pose.pose.orientation.w;

        geometry_msgs::Twist start_twist;
        start_twist.linear.x =msg->twist.twist.linear.x;
        start_twist.linear.y = msg->twist.twist.linear.y;
        start_twist.linear.z = msg->twist.twist.linear.z;
        start_twist.angular.x = msg->twist.twist.angular.x;
        start_twist.angular.y =msg->twist.twist.angular.y;
        start_twist.angular.z = msg->twist.twist.angular.z;

        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = model_name;
        modelstate.reference_frame = (std::string) "world";
        modelstate.pose = start_pose;
        modelstate.twist = start_twist;
  
   	model_state_pub.publish(modelstate);

  }

 
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "ModelState");
  ros::NodeHandle n;
  double freq = 30.0;

  ros::Rate loop_rate(freq);

  ModelState lstopc(n);
while(ros::ok())
{

  ros::spinOnce();
  loop_rate.sleep();

}  

  
  return 0;
}
