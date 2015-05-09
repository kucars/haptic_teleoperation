/***************************************************************************
* Copyright (C) 2013 - 2014 by                                             *
* Rui Figueiredo, Khalifa University Robotics Institute KURI               *
* <rui.defigueiredo@kustar.ac.ae>                                          *
*                                                                          *
* 									   *
* This program is free software; you can redistribute it and/or modify     *
* it under the terms of the GNU General Public License as published by     *
* the Free Software Foundation; either version 2 of the License, or        *
* (at your option) any later version. 					   *
* 									   *
* This program is distributed in the hope that it will be useful, 	   *
* but WITHOUT ANY WARRANTY; without even the implied warranty of 	   *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 		   *
* GNU General Public License for more details. 				   *
* 									   *
* You should have received a copy of the GNU General Public License 	   *
* along with this program; if not, write to the 			   *
* Free Software Foundation, Inc., 					   *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA. 		   *
***************************************************************************/
#ifndef CONTROLLER_
#define CONTROLLER_
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include <Eigen/Eigen>
#include <tf/transform_listener.h>
#include <phantom_omni/OmniFeedback.h>
#include <phantom_omni/PhantomButtonEvent.h>
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>
#include <haptic_teleoperation/MasterControllerConfig.h>
#include <haptic_teleoperation/SlaveControllerConfig.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
const double PI=3.14159265359;

#define BILLION 1000000000

class Controller
{	
public:
    // ros
    ros::NodeHandle n;
    ros::NodeHandle n_priv;

    ros::Publisher cmd_pub;
    ros::Publisher omni_pub;

    ros::Publisher lock_pub;
    ros::Publisher haptic_pub ;
    ros::Subscriber master_sub;
    ros::Subscriber slave_sub;
    ros::Subscriber button_sub;
    ros::Subscriber force_feedback_sub ;
    ros::Subscriber navedata;
    ros::Subscriber collision_flag;


    //ros::Subscriber velocity_cmd_sub;
    // ros::Publisher velocity_cmd_pub;
    // params
    double freq;
    Eigen::Matrix<double,6,1> Kp;
    Eigen::Matrix<double,6,1> Kd;
    Eigen::Matrix<double,6,1> Bd;
    Eigen::Matrix<double,6,1> Fp;
    Eigen::Matrix<double,6,6> lambda;


    Eigen::Matrix<double,6,1> previous_pose_master;
    Eigen::Matrix<double,6,1> current_pose_master;

    Eigen::Matrix<double,6,1> previous_pose_master_scaled;
    Eigen::Matrix<double,6,1> current_pose_master_scaled;

    Eigen::Matrix<double,6,1> previous_pose_slave;
    Eigen::Matrix<double,6,1> current_pose_slave;

    Eigen::Matrix<double,6,1> previous_pose_slave_scaled;
    Eigen::Matrix<double,6,1> current_pose_slave_scaled;

    Eigen::Matrix<double,6,1> current_velocity_master;
    Eigen::Matrix<double,6,1> current_velocity_slave;

    Eigen::Matrix<double,6,1> current_velocity_master_scaled;
    Eigen::Matrix<double,6,1> current_velocity_slave_scaled;

    Eigen::Matrix<double,6,1> master_min;
    Eigen::Matrix<double,6,1> master_max;
    Eigen::Matrix<double,6,1> master_len;

    Eigen::Matrix<double,6,1> slave_min;
    Eigen::Matrix<double,6,1> slave_max;
    Eigen::Matrix<double,6,1> slave_len;

    Eigen::Matrix<double,6,1> slave_velocity_min;
    Eigen::Matrix<double,6,1> slave_velocity_max;

    ros::Time previous_time;

    double yaw_master_joint_previous;
    double yaw_slave_previous;

    // aux
    bool master_new_readings;
    bool slave_new_readings;
    bool control_event;
    bool linear_button_pressed;
    bool angular_button_pressed;
    bool init_slave_readings;

    tf::TransformListener listener;
    tf::StampedTransform transform;


    Controller()
    {};

    ~Controller()
    {};

    Controller(ros::NodeHandle & n_,
               double freq_,
               Eigen::Matrix<double,6,1> Kp_,
               Eigen::Matrix<double,6,1> Kd_,
               Eigen::Matrix<double,6,1> Bd_,
               Eigen::Matrix<double,6,1> Fp_,
               Eigen::Matrix<double,6,6> lambda_,
               Eigen::Matrix<double,6,1> master_min_,
               Eigen::Matrix<double,6,1> master_max_,
               Eigen::Matrix<double,6,1> slave_min_,
               Eigen::Matrix<double,6,1> slave_max_,
               Eigen::Matrix<double,6,1> slave_velocity_min_,
               Eigen::Matrix<double,6,1> slave_velocity_max_);

    // HAPTIC BUTTON
    void buttonCallback(const phantom_omni::PhantomButtonEvent::ConstPtr& button);


};

#endif
