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

#include "navigation/SlaveController.h"
#include <time.h>

SlaveController::SlaveController(ros::NodeHandle & n_,
                                 double freq_,
                                 Eigen::Matrix<double,6,1> Kp_,
                                 Eigen::Matrix<double,6,1> Kd_,
                                 Eigen::Matrix<double,6,1> Bd_,
                                 Eigen::Matrix<double,6,6> lambda_,
                                 Eigen::Matrix<double,6,1> master_to_slave_scale_,
                                 Eigen::Matrix<double,6,1> master_pose_slave_velocity_scale_,
                                 Eigen::Matrix<double,6,1> master_min_,
                                 Eigen::Matrix<double,6,1> master_max_,
                                 Eigen::Matrix<double,6,1> slave_min_,
                                 Eigen::Matrix<double,6,1> slave_max_,
                                 Eigen::Matrix<double,6,1> slave_velocity_min_,
                                 Eigen::Matrix<double,6,1> slave_velocity_max_) :
    master_to_slave_scale(master_to_slave_scale_),
    master_pose_slave_velocity_scale(master_pose_slave_velocity_scale_),
    Controller(n_,freq_, Kp_, Kd_, Bd_, lambda_, master_min_, master_max_, slave_min_, slave_max_, slave_velocity_min_, slave_velocity_max_)
{
    initParams();
    slave_callback_type = boost::bind(&SlaveController::paramsCallback, this, _1, _2);
    slave_server.setCallback(slave_callback_type);

    // Feedback publish
    cmd_pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);

    // Master joint states subscriber
    master_sub = n.subscribe<sensor_msgs::JointState>("/omni1_joint_states", 1, &SlaveController::masterJointsCallback, this);

    // Slave pose and velocity subscriber
    slave_sub = n.subscribe("/RosAria/pose", 1, &SlaveController::slaveOdometryCallback, this);
}

void SlaveController::initParams()
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
    Eigen::Matrix<double,6,1> slave_min;
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
    Eigen::Matrix<double,6,1> slave_max;
    slave_max <<  slave_max_x,
            slave_max_y,
            slave_max_z,
            slave_max_roll,
            slave_max_pitch,
            slave_max_yaw;

    Eigen::Matrix<double,6,1> slave_size=slave_max-slave_min;

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
    Eigen::Matrix<double,6,1> master_min;
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
    Eigen::Matrix<double,6,1> master_max;
    master_max << master_max_x,
            master_max_y,
            master_max_z,
            master_max_roll,
            master_max_pitch,
            master_max_yaw;

    Eigen::Matrix<double,6,1> master_size=master_max-master_min;

    master_to_slave_scale << fabs(slave_size(0,0)/master_size(0,0)),
            fabs(slave_size(1,0)/master_size(1,0)),
            fabs(slave_size(2,0)/master_size(2,0)),
            fabs(slave_size(3,0)/master_size(3,0)),
            fabs(slave_size(4,0)/master_size(4,0)),
            fabs(slave_size(5,0)/master_size(5,0));

}

void SlaveController::paramsCallback(navigation::SlaveControllerConfig &config, uint32_t level)
{
    ROS_INFO_STREAM("Slave PID reconfigure Request ->"  << " kp_x:" << config.kp_x
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

    lambda << config.lambda_x, 0, 0, 0 ,0, 0,
            0, config.lambda_y, 0, 0, 0, 0,
            0, 0, config.lambda_z, 0, 0, 0,
            0, 0, 0, config.lambda_roll, 0, 0,
            0, 0, 0, 0, config.lambda_pitch, 0,
            0, 0, 0, 0, 0, config.lambda_yaw;
}



// MASTER MEASUREMENTS
void SlaveController::masterJointsCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
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
        ROS_INFO("ANGULAR PRESSED");
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

    double x_master_scaled=(-x_master-master_max(0,0))*master_to_slave_scale(0,0)-slave_min(0,0); // mirrored
    double y_master_scaled=(-y_master-master_max(1,0))*master_to_slave_scale(1,0)-slave_min(1,0); // mirrored
    double z_master_scaled=(z_master-master_min(2,0))*master_to_slave_scale(2,0)-slave_min(2,0);;

    double yaw_master_scaled=yaw_master*master_to_slave_scale(5,0);

    //std::cout << current_pose_master.transpose() << std::endl;
    // Pose master
    current_pose_master << x_master_scaled, y_master_scaled, z_master_scaled, 0.0, 0.0, yaw_master_scaled;
    // Velocity master
    timespec current_time;
    clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);
    double period = (current_time.tv_sec - previous_time.tv_sec) + (double)(current_time.tv_nsec - previous_time.tv_nsec) / (double)BILLION;

    current_velocity_master(0,0)=(-x_master+master_max(0,0))*master_pose_slave_velocity_scale(0,0)+slave_velocity_min(0,0);
    current_velocity_master(1,0)=(-y_master+master_max(1,0))*master_pose_slave_velocity_scale(1,0)+slave_velocity_min(1,0);
    current_velocity_master(2,0)=( z_master-master_min(2,0))*master_pose_slave_velocity_scale(2,0)+slave_velocity_min(2,0);
    current_velocity_master(3,0)=( current_pose_master(3,0)-previous_pose_master(3,0))/period;
    current_velocity_master(4,0)=( current_pose_master(4,0)-previous_pose_master(4,0))/period;
    current_velocity_master(5,0)=( current_pose_master(5,0)-previous_pose_master(5,0))/period;

    previous_time=current_time;
    //std::cout << current_pose_master.transpose() << std::endl;
    master_new_readings=true;
    feedback();
    previous_pose_master=current_pose_master;
}

// SLAVE MEASUREMENTS
void SlaveController::slaveOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Pose slave
    Eigen::Matrix<double,3,1> euler=Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                       msg->pose.pose.orientation.x,
                                                       msg->pose.pose.orientation.y,
                                                       msg->pose.pose.orientation.z).matrix().eulerAngles(2, 1, 0);
    double yaw = euler(0,0);
    double pitch = euler(1,0);
    double roll = euler(2,0);

    current_pose_slave << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z,
            roll,
            pitch,
            yaw;

    current_velocity_slave << msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z,
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z;

    //std::cout << "current vel:"<<current_velocity_slave.transpose() << std::endl;
    slave_new_readings=true;
    feedback();
    previous_pose_slave=current_pose_slave;

}

void SlaveController::feedback()
{
    geometry_msgs::Twist twist_msg;
    if(control_event)
    {
        //Eigen::Matrix<double,6,1> r=current_velocity_master+lambda*current_pose_master;
        Eigen::Matrix<double,6,1> r=current_velocity_master;
        Eigen::Matrix<double,6,6> feeback_matrix =
                (current_pose_master - current_pose_slave)* Kp.transpose() +
                (r - current_velocity_slave)              * Kd.transpose() +
                (  - current_velocity_master)             * Bd.transpose();

        //std::cout << "kp error:"   << (current_pose_master(5,0) -  current_pose_slave(5,0)) << std::endl;
        //std::cout << "kd error:"   << (current_velocity_master(5,0) -  current_velocity_slave(5,0)) << std::endl;

        //std::cout << "slave velocity error:"   << current_velocity_master(5,0) << " "  << current_velocity_slave(5,0) << feeback_matrix(5,5)<< std::endl;

        //std::cout << "position error:" << (current_pose_slave - current_pose_master).norm() << std::endl;
        //std::cout << "slave current velocity:" << current_velocity_slave.transpose() << std::endl;
        //std::cout << "slave velocity error:"   << (current_velocity_master -  current_pose_slave).norm() << std::endl;

        twist_msg.linear.x=feeback_matrix(0,0);
        twist_msg.linear.y=feeback_matrix(1,1);
        twist_msg.linear.z=feeback_matrix(2,2);
        twist_msg.angular.z=feeback_matrix(5,5);

        master_new_readings=false;
        slave_new_readings=false;
    }

    cmd_pub.publish(twist_msg);

}
