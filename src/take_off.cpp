/*
Parker Conroy
Algorithmic Robotics Lab @ University of Utah

This program launches the AR Drone.
It is intended as a simple example for those starting with the AR Drone platform.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
std_msgs::Empty emp_msg;
std_srvs::Empty empty;

int main(int argc, char** argv)
{

    ROS_INFO("Flying ARdrone");
    ros::init(argc, argv,"ARDrone_test");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);
    ros::Publisher pub_empty;
    ros::ServiceClient flat_trim_client = node.serviceClient<std_srvs::Empty>("/ardrone/flattrim");
    ROS_INFO("sending flat trim service request");


    pub_empty = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */

    while (ros::ok())
    {
        std::cout << "Ros::Ok" << std::endl ;
        double time_start=(double)ros::Time::now().toSec();
        flat_trim_client.call(empty);
        flat_trim_client.call(empty);
        while ((double)ros::Time::now().toSec()< time_start+10.0) /* Send command for five seconds*/
        {
            std::cout << "NODE" << std::endl ;
            std::cout << "emp_msg" << emp_msg << std::endl ;
           // flat_trim_client.call(empty);
            pub_empty.publish(emp_msg); /* launches the drone */
            ros::spinOnce();
         // loop_rate.sleep();
        }//time loop
        ROS_INFO("ARdrone launched");
        exit(0);
    }//ros::ok loop

}//main

