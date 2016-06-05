/***************************************************************************
* Copyright (C) 2013 - 2014 by                                             *
* Reem Ashour, Khalifa University Robotics Institute KURI               *
* <reem.ashour@kustar.ac.ae>                                          *
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

#include "haptic_teleoperation/MasterController.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

Eigen::Matrix<double,6,1> force_auto ;
nav_msgs::Odometry haptic_position ;
double nowTime ;
double preTime ;


MasterController::MasterController(ros::NodeHandle & n_,
                                   double freq_,
                                   Eigen::Matrix<double,6,1> Kp_,
                                   Eigen::Matrix<double,6,1> Kd_,
                                   Eigen::Matrix<double,6,1> Bd_,
                                   Eigen::Matrix<double,6,1> Fp_,
                                   Eigen::Matrix<double,6,6> lambda_,
                                   Eigen::Matrix<double,6,1> slave_to_master_scale_,
                                   Eigen::Matrix<double,6,1> slave_velocity_master_pose_scale_,
                                   Eigen::Matrix<double,6,1> master_min_,
                                   Eigen::Matrix<double,6,1> master_max_,
                                   Eigen::Matrix<double,6,1> slave_min_,
                                   Eigen::Matrix<double,6,1> slave_max_,
                                   Eigen::Matrix<double,6,1> slave_velocity_min_,
                                   Eigen::Matrix<double,6,1> slave_velocity_max_) :
    slave_to_master_scale(slave_to_master_scale_),
    slave_velocity_master_pose_scale(slave_velocity_master_pose_scale_),
    Controller(n_,freq_, Kp_, Kd_, Bd_,Fp_, lambda_, master_min_, master_max_, slave_min_, slave_max_,slave_velocity_min_,slave_velocity_max_)
{

    initParams();
    master_callback_type = boost::bind(&MasterController::paramsCallback, this, _1, _2);
    master_server.setCallback(master_callback_type);

    // Feedback publish to the haptic device
    omni_pub = n_.advertise<phantom_omni::OmniFeedback>("/omni1_force_feedback", 1);
    lock_pub = n_.advertise<std_msgs::Bool>("/lock", 1);

    // haptic position publisher in linear coordinates
    haptic_pub = n_.advertise<nav_msgs::Odometry>("haptic_position_pub",1) ;

    // Master joint states subscriber
    master_sub = n_.subscribe<sensor_msgs::JointState>("/omni1_joint_states", 1, &MasterController::masterJointsCallback, this);

    // Slave pose and velocity subscriber from ( gazebo or the real robot)
<<<<<<< HEAD
    //slave_sub = n.subscribe("/Pioneer3AT/pose", 1, &MasterController::slaveOdometryCallback, this); // for pioneer
  //  slave_sub = n.subscribe("/pose", 1, &MasterController::slaveOdometryCallback, this); // for airdrone
    //slave_sub = n_.subscribe("/mavros/vision_pose/pose", 1, &MasterController::slaveOdometryCallback, this); // for airdrone
    slave_sub = n_.subscribe("/ground_truth/state" , 1 , &MasterController::slaveOdometryCallback, this);
=======
    slave_sub = n_.subscribe("/mavros/vision_pose/pose", 1, &MasterController::slaveOdometryCallback, this); // for airdrone
>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3


    // subscribe for the environmental force from the potential fieldFp
    force_feedback_sub  = n_.subscribe("/virtual_force_feedback" , 1, &MasterController::getforce_feedback   , this);


}

void MasterController::initParams()
{
    // parameters
    double freq;
    double kp_x;
    double kp_y;
    double kp_z;
    double kp_roll;
    double kp_pitch;
    double kp_yaw;

    double kd_x;
    double kd_y;
    double kd_z;
    double kd_roll;
    double kd_pitch;
    double kd_yaw;

    double fp_x;
    double fp_y;
    double fp_z;
    double fp_roll;
    double fp_pitch;
    double fp_yaw;

    //initialize operational parameters
    n_priv.param<double>("frequency", freq, 10.0);


    double slave_min_x;
    double slave_min_y;
    double slave_min_z;
    double slave_min_roll;
    double slave_min_pitch;
    double slave_min_yaw;
    n_priv.param<double>("slave_min_x",     slave_min_x, 1.0);
    n_priv.param<double>("slave_min_y",     slave_min_y, 1.0);
    n_priv.param<double>("slave_min_z",     slave_min_z, 1.0);
    n_priv.param<double>("slave_min_roll",  slave_min_roll, 1.0);
    n_priv.param<double>("slave_min_pitch", slave_min_pitch, 1.0);
    n_priv.param<double>("slave_min_yaw",   slave_min_yaw, 1.0);

    slave_min <<  slave_min_x,
            slave_min_y,
            slave_min_z,
            slave_min_roll,
            slave_min_pitch,
            slave_min_yaw;

    double slave_max_x;
    double slave_max_y;
    double slave_max_z;
    double slave_max_roll;
    double slave_max_pitch;
    double slave_max_yaw;
    n_priv.param<double>("slave_max_x",     slave_max_x, 1.0);
    n_priv.param<double>("slave_max_y",     slave_max_y, 1.0);
    n_priv.param<double>("slave_max_z",     slave_max_z, 1.0);
    n_priv.param<double>("slave_max_roll",  slave_max_roll,  1.0);
    n_priv.param<double>("slave_max_pitch", slave_max_pitch, 1.0);
    n_priv.param<double>("slave_max_yaw",   slave_max_yaw,   1.0);

    slave_max <<  slave_max_x,
            slave_max_y,
            slave_max_z,
            slave_max_roll,
            slave_max_pitch,
            slave_max_yaw;

    slave_len=slave_max-slave_min;

    double master_min_x;
    double master_min_y;
    double master_min_z;
    double master_min_roll;
    double master_min_pitch;
    double master_min_yaw;
    n_priv.param<double>("master_min_x",     master_min_x,     1.0);
    n_priv.param<double>("master_min_y",     master_min_y,     1.0);
    n_priv.param<double>("master_min_z",     master_min_z,     1.0);
    n_priv.param<double>("master_min_roll",  master_min_roll,  1.0);
    n_priv.param<double>("master_min_pitch", master_min_pitch, 1.0);
    n_priv.param<double>("master_min_yaw",   master_min_yaw,   1.0);

    master_min << master_min_x,
            master_min_y,
            master_min_z,
            master_min_roll,
            master_min_pitch,
            master_min_yaw;

    double master_max_x;
    double master_max_y;
    double master_max_z;
    double master_max_roll;
    double master_max_pitch;
    double master_max_yaw;
    n_priv.param<double>("master_max_x",     master_max_x,     1.0);
    n_priv.param<double>("master_max_y",     master_max_y,     1.0);
    n_priv.param<double>("master_max_z",     master_max_z,     1.0);
    n_priv.param<double>("master_max_roll",  master_max_roll,  1.0);
    n_priv.param<double>("master_max_pitch", master_max_pitch, 1.0);
    n_priv.param<double>("master_max_yaw",   master_max_yaw,   1.0);

    master_max << master_max_x,
            master_max_y,
            master_max_z,
            master_max_roll,
            master_max_pitch,
            master_max_yaw;

    master_len=master_max-master_min;

    slave_to_master_scale << fabs(master_len(0,0)/slave_len(0,0)),
            fabs(master_len(1,0)/slave_len(1,0)),
            fabs(master_len(2,0)/slave_len(2,0)),
            fabs(master_len(3,0)/slave_len(3,0)),
            fabs(master_len(4,0)/slave_len(4,0)),
            fabs(master_len(5,0)/slave_len(5,0));



}

void MasterController::paramsCallback(haptic_teleoperation::MasterControllerConfig &config, uint32_t level)
{
    ROS_DEBUG_STREAM("Master PID reconfigure Request ->" << " kp_x:" << config.kp_x
                     << " kp_y:" << config.kp_y
                     << " kp_z:" << config.kp_z
                     << " kd_x:" << config.kd_x
                     << " kd_y:" << config.kd_y
                     << " kd_z:" << config.kd_z);

    Kp << config.kp_x,
            config.kp_y,
            config.kp_z,
            config.kp_roll,
            config.kp_pitch,
            config.kp_yaw;

    Kd << config.kd_x,
            config.kd_y,
            config.kd_z,
            config.kd_roll,
            config.kd_pitch,
            config.kd_yaw;

    Bd << config.bd_x,
            config.bd_y,
            config.bd_z,
            config.bd_roll,
            config.bd_pitch,
            config.bd_yaw;
    Fp << config.fp_x, config.fp_y, config.fp_z, config.fp_roll,config.fp_pitch, config.fp_yaw;

    lambda << config.lambda_x, 0, 0, 0 ,0, 0,
            0, config.lambda_y, 0, 0, 0, 0,
            0, 0, config.lambda_z, 0, 0, 0,
            0, 0, 0, config.lambda_roll, 0, 0,
            0, 0, 0, 0, config.lambda_pitch, 0,
            0, 0, 0, 0, 0, config.lambda_yaw;



    //slave_to_master_scale=Eigen::Matrix<double,3,1> (fabs(config.master_size.x/config.slave_size.x), fabs(config.master_size.y/config.slave_size.y), fabs(config.master_size.z/config.slave_size.z));
}

//void MasterController::feedbackCallback(const geometry_msgs::Point::ConstPtr& force)
//{
////    Eigen::Matrix<double,6,1> feedback_fore ;
////    feedback_fore(1) = force.x() ;
////    feedback_fore(2) = force.y() ;
////    feedback_fore[3][1] = force.z() ;
////}
//}
void MasterController::getforce_feedback(const geometry_msgs::PoseStamped::ConstPtr& force)
{

    force_auto <<  force->pose.position.x,
            force->pose.position.y,
            force->pose.position.z,
            0,
            0,
            0;
    std::cout << "force_x" << force->pose.position.x << std::endl ;
    std::cout << "force_y" << force->pose.position.y << std::endl ;
    std::cout << "force_z" << force->pose.position.z << std::endl ;
}


// MASTER MEASUREMENTS
void MasterController::masterJointsCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{

    double x_master=(master_max(0,0)-master_min(0,0))/2.0+master_min(0,0);
    double y_master=(master_max(1,0)-master_min(1,0))/2.0+master_min(1,0);
    double z_master=(master_max(2,0)-master_min(2,0))/2.0+master_min(2,0);
    double yaw_master_joint=joint_states->position[5];
    double yaw_master=0.0;
    if(linear_button_pressed)
    {
        try
        {
            listener.lookupTransform("/base", "/wrist2",ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        x_master=transform.getOrigin().x();
        if(x_master<master_min(0,0))
        {
            x_master=master_min(0,0);
        }
        else if(x_master>master_max(0,0))
        {
            x_master=master_max(0,0);
        }

        y_master=transform.getOrigin().y();
        if(y_master<master_min(1,0))
        {
            y_master=master_min(1,0);
        }
        else if(y_master>master_max(1,0))
        {
            y_master=master_max(1,0);
        }

        z_master=transform.getOrigin().z();
        if(z_master<master_min(2,0))
        {
            z_master=master_min(2,0);
        }
        else if(z_master>master_max(2,0))
        {
            z_master=master_max(2,0);
        }
    }
    if(angular_button_pressed)
    {
        // Wrist3 controls angular speed
        if(yaw_master_joint<master_min(5,0))
        {
            yaw_master_joint=master_min(5,0);
        }
        else if(yaw_master_joint>master_max(5,0))
        {
            yaw_master_joint=master_max(5,0);
        }
        yaw_master=yaw_master_joint-yaw_master_joint_previous; // delta q - desired position
    }
    yaw_master_joint_previous=yaw_master_joint;

    yaw_master_joint_previous=yaw_master_joint;
    ros::Time current_time=ros::Time::now();
    double period = current_time.toSec()-previous_time.toSec();
    previous_time=current_time;
    //std::cout << "period:"<< period << std::endl;
    // Pose master
    // x and y are mirrored
    // angles are relative
    current_pose_master <<
                           (-x_master + master_min(0,0)+master_max(0,0)),
            (-y_master + master_min(1,0)+master_max(1,0)),
            z_master,
            0.0,
            0.0,
            yaw_master;

    haptic_position.header.stamp =  ros::Time::now();
    haptic_pub.publish(haptic_position);
    current_velocity_master=(current_pose_master-previous_pose_master)/period;
    haptic_position.pose.pose.position.x = -x_master + master_min(0,0)+master_max(0,0);
    haptic_position.pose.pose.position.y = -y_master + master_min(1,0)+master_max(1,0);
    haptic_position.pose.pose.position.z = z_master  ;

    // haptic_position.twist.twist.linear.x = current_velocity_master(0,0) ;
    // haptic_position.twist.twist.linear.x = current_velocity_master(0,1) ;
    // haptic_position.twist.twist.linear.x = current_velocity_master(0,2) ;
    master_new_readings=true;
    //feedback();
    previous_pose_master=current_pose_master;

}
<<<<<<< HEAD
void MasterController::slaveOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Pose slave
    Eigen::Matrix<double,3,1> euler=Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                       msg->pose.pose.orientation.x,
                                                       msg->pose.pose.orientation.y,
                                                       msg->pose.pose.orientation.z).matrix().eulerAngles(2, 1, 0);
    double yaw = euler(0,0);
    double pitch = euler(1,0);
    double roll = euler(2,0);
=======

// SLAVE MEASUREMENTS


void MasterController::slaveOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg){


    // Pose slave
//    Eigen::Matrix<double,3,1> euler=Eigen::Quaterniond(msg->pose.orientation.w,
//                                                       msg->pose.orientation.x,
//                                                       msg->pose.orientation.y,
//                                                       msg->pose.orientation.z).matrix().eulerAngles(2, 1, 0);
//    double yaw = euler(0,0);
//    double pitch = euler(1,0);
//    double roll = euler(2,0);
    tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    std::cout << "YAW = " << yaw * 180 /3.4 << std::endl ;

>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3
    if(!init_slave_readings)
    {
        previous_pose_slave << msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z,
                roll-previous_pose_slave(3,0),
                pitch-previous_pose_slave(4,0),
                yaw; // should be relative
        // std::cout << "previous_pose_slave:" << previous_pose_slave(5,0) << " yaw:" << yaw << std::endl;
        // std::cout << "yaw:" << yaw << " yaw previous:" << yaw_slave_previous << std::endl;
        yaw_slave_previous=yaw;
        init_slave_readings=true;
        return;
    }
    else
    {
<<<<<<< HEAD
        // lastPositionUpdate = ros::Time::now().toSec();
=======
>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3
        current_pose_slave << msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z,
                roll-previous_pose_slave(3,0),
                pitch-previous_pose_slave(4,0),
                yaw_slave_previous; // should be relative
        // std::cout << "current_pose_slave:" << current_pose_slave(5,0) << " yaw_slave_previous:" << yaw_slave_previous << std::endl;
        // std::cout << current_pose_slave(0,0) << std::endl ;
        // std::cout << current_pose_slave(1,0) << std::endl ;
        // std::cout << current_pose_slave(2,0) << std::endl ;
        // std::cout << current_pose_slave(3,0) << std::endl ;
        // std::cout << current_pose_slave(4,0) << std::endl ;
        // std::cout << current_pose_slave(5,0) << std::endl ;
        // double test = current_pose_slave(5,0) ;
        // std::cout << "yaw:" << yaw << " yaw previous:" << yaw_slave_previous << std::endl;
        // std::cout << "current_pose_slave:" << test << " previous_pose_slave previous:" << previous_pose_slave(5,0) << std::endl;
        // std::cout << "yaw TO DEG:" << yaw*RAD_TO_DEG << " yaw previous TO DEG:" << yaw_slave_previous * RAD_TO_DEG << std::endl;
        // std::cout << "current_pose_slave TO DEG (((:" << current_pose_slave(5,0)*RAD_TO_DEG << " previous_pose_slave previous TO DEG )))):" << previous_pose_slave(5,0) * RAD_TO_DEG << std::endl;
        yaw_slave_previous=yaw;
    }
    current_velocity_slave << msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z,
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z;
    slave_new_readings=true;
    feedback();
    previous_pose_slave=current_pose_slave;
}
// SLAVE MEASUREMENTS
//void MasterController::slaveOdometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){


//    // Pose slave
////    Eigen::Matrix<double,3,1> euler=Eigen::Quaterniond(msg->pose.orientation.w,
////                                                       msg->pose.orientation.x,
////                                                       msg->pose.orientation.y,
////                                                       msg->pose.orientation.z).matrix().eulerAngles(2, 1, 0);
////    double yaw = euler(0,0);
////    double pitch = euler(1,0);
////    double roll = euler(2,0);
//    tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
//    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

//    std::cout << "YAW = " << yaw * 180 /3.4 << std::endl ;

//    if(!init_slave_readings)
//    {
//        previous_pose_slave << msg->pose.position.x,
//                msg->pose.position.y,
//                msg->pose.position.z,
//                roll-previous_pose_slave(3,0),
//                pitch-previous_pose_slave(4,0),
//                yaw_slave_previous; // should be relative
//        preTime = ros::Time::now().toSec() ;
//        yaw_slave_previous=yaw;
//        init_slave_readings=true;
//        return;
//    }
//    else
//    {
//        current_pose_slave << msg->pose.position.x,
//                msg->pose.position.y,
//                msg->pose.position.z,
//                roll-previous_pose_slave(3,0),
//                pitch-previous_pose_slave(4,0),
//                yaw-yaw_slave_previous; // should be relative
//        nowTime = ros::Time::now().toSec() ;

//        //std::cout << "yaw:" << yaw << " yaw previous:" << yaw_slave_previous << std::endl;

//        yaw_slave_previous=yaw;
//    }


//    current_pose_slave_scaled(0,0)=(current_pose_slave(0,0)-slave_min(0,0))*slave_to_master_scale(0,0) + master_min(0,0);
//    current_pose_slave_scaled(1,0)=(current_pose_slave(1,0)-slave_min(1,0))*slave_to_master_scale(1,0) + master_min(1,0);
//    current_pose_slave_scaled(2,0)=(current_pose_slave(2,0)-slave_min(2,0))*slave_to_master_scale(2,0) + master_min(2,0);
//    current_pose_slave_scaled(3,0)=(current_pose_slave(3,0)-slave_min(3,0))*slave_to_master_scale(3,0) + master_min(3,0);
//    current_pose_slave_scaled(4,0)=(current_pose_slave(4,0)-slave_min(4,0))*slave_to_master_scale(4,0) + master_min(4,0);
//    current_pose_slave_scaled(5,0)=(current_pose_slave(5,0)-slave_min(5,0))*slave_to_master_scale(5,0) + master_min(5,0);

//    current_velocity_slave << ((current_pose_slave(0,0)-previous_pose_slave(0,0)/(nowTime - preTime))-slave_velocity_min(0,0)) * slave_velocity_master_pose_scale(0,0) + master_min(0,0),
//            ((current_pose_slave(1,0)-previous_pose_slave(1,0)/(nowTime - preTime))-slave_velocity_min(1,0)) * slave_velocity_master_pose_scale(1,0) + master_min(1,0),
//            (current_pose_slave(2,0)-previous_pose_slave(2,0)/(nowTime - preTime)-slave_velocity_min(2,0)) * slave_velocity_master_pose_scale(2,0)  + master_min(2,0),
//            (current_pose_slave(3,0)-previous_pose_slave(3,0)/(nowTime - preTime)-slave_velocity_min(3,0)) * slave_velocity_master_pose_scale(3,0) + master_min(3,0),
//            (current_pose_slave(4,0)-previous_pose_slave(4,0)/(nowTime - preTime)-slave_velocity_min(4,0)) * slave_velocity_master_pose_scale(4,0) + master_min(4,0),
//            (current_pose_slave(5,0)-previous_pose_slave(5,0)/(nowTime - preTime)-slave_velocity_min(5,0)) * slave_velocity_master_pose_scale(5,0) + master_min(5,0);

//    std::cout << "current_velocity_slave:"<<current_velocity_slave(0,0) << " " << master_min(0,0)<< " " << master_max(0,0)<< std::endl;
//   // std::cout << "current_velocity_slave:"<<slave_velocity_min.transpose() << std::endl;


//    slave_new_readings=true;
//    feedback();
//    previous_pose_slave=current_pose_slave;
//}

/* this function is used with mavros package */
//void MasterController::slaveOdometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){


//    // Pose slave
////    Eigen::Matrix<double,3,1> euler=Eigen::Quaterniond(msg->pose.orientation.w,
////                                                       msg->pose.orientation.x,
////                                                       msg->pose.orientation.y,
////                                                       msg->pose.orientation.z).matrix().eulerAngles(2, 1, 0);
////    double yaw = euler(0,0);
////    double pitch = euler(1,0);
////    double roll = euler(2,0);
//    tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
//    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

//    std::cout << "YAW = " << yaw * 180 /3.4 << std::endl ;

//    if(!init_slave_readings)
//    {
//        previous_pose_slave << msg->pose.position.x,
//                msg->pose.position.y,
//                msg->pose.position.z,
//                roll-previous_pose_slave(3,0),
//                pitch-previous_pose_slave(4,0),
//                yaw_slave_previous; // should be relative
//        preTime = ros::Time::now().toSec() ;
//        yaw_slave_previous=yaw;
//        init_slave_readings=true;
//        return;
//    }
//    else
//    {
//        current_pose_slave << msg->pose.position.x,
//                msg->pose.position.y,
//                msg->pose.position.z,
//                roll-previous_pose_slave(3,0),
//                pitch-previous_pose_slave(4,0),
//                yaw-yaw_slave_previous; // should be relative
//        nowTime = ros::Time::now().toSec() ;

//        //std::cout << "yaw:" << yaw << " yaw previous:" << yaw_slave_previous << std::endl;

//        yaw_slave_previous=yaw;
//    }


//    current_pose_slave_scaled(0,0)=(current_pose_slave(0,0)-slave_min(0,0))*slave_to_master_scale(0,0) + master_min(0,0);
//    current_pose_slave_scaled(1,0)=(current_pose_slave(1,0)-slave_min(1,0))*slave_to_master_scale(1,0) + master_min(1,0);
//    current_pose_slave_scaled(2,0)=(current_pose_slave(2,0)-slave_min(2,0))*slave_to_master_scale(2,0) + master_min(2,0);
//    current_pose_slave_scaled(3,0)=(current_pose_slave(3,0)-slave_min(3,0))*slave_to_master_scale(3,0) + master_min(3,0);
//    current_pose_slave_scaled(4,0)=(current_pose_slave(4,0)-slave_min(4,0))*slave_to_master_scale(4,0) + master_min(4,0);
//    current_pose_slave_scaled(5,0)=(current_pose_slave(5,0)-slave_min(5,0))*slave_to_master_scale(5,0) + master_min(5,0);

//    current_velocity_slave << ((current_pose_slave(0,0)-previous_pose_slave(0,0)/(nowTime - preTime))-slave_velocity_min(0,0)) * slave_velocity_master_pose_scale(0,0) + master_min(0,0),
//            ((current_pose_slave(1,0)-previous_pose_slave(1,0)/(nowTime - preTime))-slave_velocity_min(1,0)) * slave_velocity_master_pose_scale(1,0) + master_min(1,0),
//            (current_pose_slave(2,0)-previous_pose_slave(2,0)/(nowTime - preTime)-slave_velocity_min(2,0)) * slave_velocity_master_pose_scale(2,0)  + master_min(2,0),
//            (current_pose_slave(3,0)-previous_pose_slave(3,0)/(nowTime - preTime)-slave_velocity_min(3,0)) * slave_velocity_master_pose_scale(3,0) + master_min(3,0),
//            (current_pose_slave(4,0)-previous_pose_slave(4,0)/(nowTime - preTime)-slave_velocity_min(4,0)) * slave_velocity_master_pose_scale(4,0) + master_min(4,0),
//            (current_pose_slave(5,0)-previous_pose_slave(5,0)/(nowTime - preTime)-slave_velocity_min(5,0)) * slave_velocity_master_pose_scale(5,0) + master_min(5,0);

//    std::cout << "current_velocity_slave:"<<current_velocity_slave(0,0) << " " << master_min(0,0)<< " " << master_max(0,0)<< std::endl;
//   // std::cout << "current_velocity_slave:"<<slave_velocity_min.transpose() << std::endl;


//    slave_new_readings=true;
//    feedback();
//    previous_pose_slave=current_pose_slave;
//}


void MasterController::feedback()
{
    Eigen::Matrix<double,6,1> Km_1;
    Eigen::Matrix<double,6,1> Km_2;
    Eigen::Matrix<double,6,6> Fe ;

    Km_1 << 1.5, 1.5, 1.5, 1.5, 1.5, 1.5  ;
    Km_2 << 1.5, 1.5, 1.5, 1.5, 1.5, 1.5  ;

    phantom_omni::OmniFeedback force_msg;
    Fe = Fp * force_auto.transpose()  ;
    Eigen::Matrix<double,6,6> feedback_matrix;

    Eigen::Matrix<double,6,6> kx ;
    kx << 0.1 ,0.0 ,0.0 ,0.0 ,0.0  ,0.0,
          0.0 ,1.0 ,0.0 ,0.0 ,0.0  ,0.0,
          0.0 ,0.0 ,1.0 ,0.0 ,0.0  ,0.0,
          0.0 ,0.0 ,0.0 ,1.0 ,0.0  ,0.0,
          0.0 ,0.0 ,0.0 ,0.0 ,1.0  ,0.0,
          0.0 ,0.0 ,0.0 ,0.0 ,0.0  ,1.0;

    feedback_matrix=-Fe;
    if(control_event )
    {

        Eigen::Matrix<double,6,1> r=current_pose_master;
        feedback_matrix += ((current_pose_slave_scaled -  current_pose_master) * Kp.transpose() +
                (current_velocity_slave -  r)                   * Kd.transpose() +
                (current_velocity_master_scaled-current_velocity_slave)*Bd.transpose() - Fe ) ;
    }

<<<<<<< HEAD
    force_msg.force.x=1.5* feedback_matrix(1,1);
    force_msg.force.y=1*feedback_matrix(2,2); // sign problem again
    force_msg.force.z=1.5* feedback_matrix(0,0); //feedback_matrix(0,0);
//    force_msg.force.x=0.0;
//    force_msg.force.y=0.0; // sign problem again
//    force_msg.force.z=0.0; //feedback_matrix(0,0);

=======
    force_msg.force.x=3* feedback_matrix(1,1);
    force_msg.force.y=0*feedback_matrix(2,2); // sign problem again
    force_msg.force.z=3* feedback_matrix(0,0); //feedback_matrix(0,0);
//   force_msg.force.x=0.0;
//   force_msg.force.y=0.0; // sign problem again
//   force_msg.force.z=0.0; //feedback_matrix(0,0);
>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3
    master_new_readings=false;
    slave_new_readings=false;
    omni_pub.publish(force_msg);

}

