#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <cmath>
#include <phantom_omni/OmniFeedback.h>
#include <dynamic_reconfigure/server.h>
#include <haptic_teleoperation/ForceFieldConfig.h>
#include <haptic_teleoperation/TwistArray.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <haptic_teleoperation/matlab_data_force.h>
#include <sstream>
#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

std::vector<Eigen::Vector3d> obstacles_positions_current;
Eigen::Matrix<double,6,1> current_pose_slave;
Eigen::Matrix<double,6,1> current_velocity_slave;
Eigen::Matrix<double,6,1> previous_pose_slave;
float Arr_x[90];
float Arr_y[90];
float Arr_z[90];
void obstacle_Callback(const sensor_msgs::PointCloud::ConstPtr& msg);
void force_x_Callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void force_y_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void force_z_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void pose_Callback(const nav_msgs::Odometry::ConstPtr& msg);
double yaw_slave_previous;

bool init_slave_readings = false;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "data_collector");
    ros::NodeHandle n;
    ros::Subscriber obstacle_readings_sub = n.subscribe("cloud", 1, obstacle_Callback );
    ros::Subscriber force_sub = n.subscribe("force_array", 1, force_x_Callback );
    ros::Subscriber pose_sub = n.subscribe("pose", 1, pose_Callback);
    //ros::Publisher collected_data_pub = n.advertise<>("collected_data", 1000);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {

        // establish the msg that we want to send
        haptic_teleoperation::matlab_data_force msg ;
        msg.header.stamp = ros::Time::now() ;
        msg.pose_x = current_pose_slave(1,1) ;
        msg.pose_y = current_pose_slave(2,1) ;
        msg.pose_z = current_pose_slave(3,1) ;
       // msg.obst_x = obstacles_positions_current(1) ;
//        msg.force_x =  *Arr_x ;
//        msg.force_y =  *Arr_y ;
//        msg.force_z =  *Arr_z ;

        // append the current position obstacles
        // append the forces ( I didnt do it yet )
        // append the pose
        //    collected_data_pub.publish() ;
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}

void obstacle_Callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    obstacles_positions_current.clear();
    for(int i=0; i< msg->points.size(); ++i)
    {
        Eigen::Vector3d obstacle(msg->points[i].x,msg->points[i].y,0.0);
        if(obstacle.norm()<4.0  && obstacle.norm()>0.2+0.01)
        {
            obstacles_positions_current.push_back(obstacle);
        }
    }
}
void force_x_Callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    int i = 0;
    // print all the remaining numbers
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        Arr_x[i] = *it;
        i++;
    }

}




void force_y_Callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    int i = 0;
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        Arr_y[i] = *it;
        i++;
    }


}
void force_z_Callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

    int i = 0;
    // print all the remaining numbers
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        Arr_z[i] = *it;
        i++;
    }

}
void pose_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Pose slave
    Eigen::Matrix<double,3,1> euler=Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                       msg->pose.pose.orientation.x,
                                                       msg->pose.pose.orientation.y,
                                                       msg->pose.pose.orientation.z).matrix().eulerAngles(2, 1, 0);
    double yaw = euler(0,0);
    double pitch = euler(1,0);
    double roll = euler(2,0);

    if(!init_slave_readings)
    {
        previous_pose_slave << msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z,
                roll-previous_pose_slave(3,0),
                pitch-previous_pose_slave(4,0),
                yaw; // should be relative
        yaw_slave_previous=yaw;
        init_slave_readings=true;
        return;
    }
    else
    {
        current_pose_slave << msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z,
                roll-previous_pose_slave(3,0),
                pitch-previous_pose_slave(4,0),
                yaw-yaw_slave_previous; // should be relative
        //std::cout << "yaw:" << yaw << " yaw previous:" << yaw_slave_previous << std::endl;

        yaw_slave_previous=yaw;
    }

    //    current_pose_slave << msg->pose.pose.position.x,
    //            msg->pose.pose.position.y,
    //            msg->pose.pose.position.z,
    //            roll-previous_pose_slave(3,0),
    //            pitch-previous_pose_slave(4,0),
    //            yaw; // should be relative




    current_velocity_slave << msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z,
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z;

    previous_pose_slave=current_pose_slave;


}
