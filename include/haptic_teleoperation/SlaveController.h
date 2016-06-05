///***************************************************************************
//* Copyright (C) 2013 - 2014 by *
//* Rui Figueiredo, Khalifa University Robotics Institute KURI *
//* <rui.defigueiredo@kustar.ac.ae> *
//* *
//* *
//* This program is free software; you can redistribute it and/or modify *
//* it under the terms of the GNU General Public License as published by *
//* the Free Software Foundation; either version 2 of the License, or *
//* (at your option) any later version. *
//* *
//* This program is distributed in the hope that it will be useful, *
//* but WITHOUT ANY WARRANTY; without even the implied warranty of *
//* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the *
//* GNU General Public License for more details. *
//* *
//* You should have received a copy of the GNU General Public License *
//* along with this program; if not, write to the *
//* Free Software Foundation, Inc., *
//* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA. *
//***************************************************************************/
//#ifndef SLAVE_CONTROLLER_
//#define SLAVE_CONTROLLER_
//#include "haptic_teleoperation/Controller.h"
//#include "ardrone_autonomy/Navdata.h"
//#include <std_msgs/Bool.h>

//class SlaveController : public Controller
//{
//public:
//    dynamic_reconfigure::Server<haptic_teleoperation::SlaveControllerConfig> slave_server;
//    dynamic_reconfigure::Server<haptic_teleoperation::SlaveControllerConfig>::CallbackType slave_callback_type;

//    SlaveController(ros::NodeHandle & n_,
//                    double freq_,
//                    Eigen::Matrix<double,6,1> Kp_,
//                    Eigen::Matrix<double,6,1> Kd_,
//                    Eigen::Matrix<double,6,1> Bd_,
//                    Eigen::Matrix<double,6,1> Fp_,
//                    Eigen::Matrix<double,6,6> lambda_,
//                    Eigen::Matrix<double,6,1> master_to_slave_scale_,
//                    Eigen::Matrix<double,6,1> master_pose_slave_velocity_scale_,
//                    Eigen::Matrix<double,6,1> master_min_,
//                    Eigen::Matrix<double,6,1> master_max_,
//                    Eigen::Matrix<double,6,1> slave_min_,
//                    Eigen::Matrix<double,6,1> slave_max_,
//                    Eigen::Matrix<double,6,1> slave_velocity_min_,
//                    Eigen::Matrix<double,6,1> slave_velocity_max_);


//    void paramsCallback(haptic_teleoperation::SlaveControllerConfig &config, uint32_t level);
//    void setfeedbackForce(Eigen::Vector3d &f) ;
//    double getfeedbackForceNorm(){
//    return feedbackForce.norm();


//    }
//private:
//    // MASTER MEASUREMENTS
//    void masterJointsCallback(const sensor_msgs::JointState::ConstPtr& joint_states);
//    void feedbackFocreCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
//    // SLAVE MEASUREMENTS
//    void slaveOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
//    //void slaveOdometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

//    void get_navdata(const ardrone_autonomy::Navdata::ConstPtr& msg);
//    bool geoFence(double timeSample , Eigen::Matrix<double,6,1> currentPose , Eigen::Matrix<double,6,6> desiredVelocity , double xBoundry , double yBoundry) ;


//    void get_inCollision(const std_msgs::Bool::ConstPtr&  _inCollision);

//    void feedback();
//    void initParams();
//    void getforce_feedback (const geometry_msgs::PoseStamped::ConstPtr & force);
//    //void getvel_feedback (const geometry_msgs::Twist::ConstPtr & force);
//    Eigen::Matrix<double,6,1> master_to_slave_scale;
//    Eigen::Matrix<double,6,1> master_pose_slave_velocity_scale;
//    Eigen::Vector3d feedbackForce;
//    double yaw,pitch,roll;

//};
//#endif


/***************************************************************************
* Copyright (C) 2013 - 2014 by *
* Rui Figueiredo, Khalifa University Robotics Institute KURI *
* <rui.defigueiredo@kustar.ac.ae> *
* *
* *
* This program is free software; you can redistribute it and/or modify *
* it under the terms of the GNU General Public License as published by *
* the Free Software Foundation; either version 2 of the License, or *
* (at your option) any later version. *
* *
* This program is distributed in the hope that it will be useful, *
* but WITHOUT ANY WARRANTY; without even the implied warranty of *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the *
* GNU General Public License for more details. *
* *
* You should have received a copy of the GNU General Public License *
* along with this program; if not, write to the *
* Free Software Foundation, Inc., *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA. *
***************************************************************************/
#ifndef SLAVE_CONTROLLER_
#define SLAVE_CONTROLLER_
#include "haptic_teleoperation/Controller.h"
#include "ardrone_autonomy/Navdata.h"
#include <std_msgs/Bool.h>

class SlaveController : public Controller
{
public:
    dynamic_reconfigure::Server<haptic_teleoperation::SlaveControllerConfig> slave_server;
    dynamic_reconfigure::Server<haptic_teleoperation::SlaveControllerConfig>::CallbackType slave_callback_type;

    SlaveController(ros::NodeHandle & n_,
                    double freq_,
                    Eigen::Matrix<double,6,1> Kp_,
                    Eigen::Matrix<double,6,1> Kd_,
                    Eigen::Matrix<double,6,1> Bd_,
                    Eigen::Matrix<double,6,1> Fp_,
                    Eigen::Matrix<double,6,6> lambda_,
                    Eigen::Matrix<double,6,1> master_to_slave_scale_,
                    Eigen::Matrix<double,6,1> master_pose_slave_velocity_scale_,
                    Eigen::Matrix<double,6,1> master_min_,
                    Eigen::Matrix<double,6,1> master_max_,
                    Eigen::Matrix<double,6,1> slave_min_,
                    Eigen::Matrix<double,6,1> slave_max_,
                    Eigen::Matrix<double,6,1> slave_velocity_min_,
                    Eigen::Matrix<double,6,1> slave_velocity_max_);


    void paramsCallback(haptic_teleoperation::SlaveControllerConfig &config, uint32_t level);
    void setfeedbackForce(Eigen::Vector3d &f) ;
    double getfeedbackForceNorm(){
    return feedbackForce.norm();


    }
private:
    // MASTER MEASUREMENTS
    void masterJointsCallback(const sensor_msgs::JointState::ConstPtr& joint_states);
    void feedbackFocreCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    // SLAVE MEASUREMENTS
    void slaveOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    //void slaveOdometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void get_navdata(const ardrone_autonomy::Navdata::ConstPtr& msg);
    bool geoFence(double timeSample , Eigen::Matrix<double,6,1> currentPose , Eigen::Matrix<double,6,6> desiredVelocity , double xBoundry , double yBoundry) ;


    void get_inCollision(const std_msgs::Bool::ConstPtr&  _inCollision);

    void feedback();
    void initParams();
    void getforce_feedback (const geometry_msgs::PoseStamped::ConstPtr & force);
    //void getvel_feedback (const geometry_msgs::Twist::ConstPtr & force);
    Eigen::Matrix<double,6,1> master_to_slave_scale;
    Eigen::Matrix<double,6,1> master_pose_slave_velocity_scale;
    Eigen::Vector3d feedbackForce;
    double yaw,pitch,roll;

};
#endif
