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

#include "haptic_teleoperation/SlaveController.h"
#include <time.h>
#include "ardrone_autonomy/Navdata.h"
#define RAD_TO_DEG 180/3.14
double battery_per ;
Eigen::Matrix<double,6,1> force_stop ;
// Eigen::Matrix<double,6,6> velocity ;

SlaveController::SlaveController(ros::NodeHandle & n_,
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
                                 Eigen::Matrix<double,6,1> slave_velocity_max_) :
    master_to_slave_scale(master_to_slave_scale_),
    master_pose_slave_velocity_scale(master_pose_slave_velocity_scale_),
    Controller(n_,freq_, Kp_, Kd_, Bd_,Fp_, lambda_, master_min_, master_max_, slave_min_, slave_max_, slave_velocity_min_, slave_velocity_max_)
{
    std::cout << " Initilization" << std::endl ;
    initParams();
    //slave_callback_type = boost::bind(&SlaveController::paramsCallback, this, _1, _2);
    //slave_server.setCallback(slave_callback_type);
    //std::cout << "velocities min " << slave_velocity_min_.transpose() << std::endl ;
    //std::cout << "velocities max " << slave_velocity_max_.transpose() << std::endl ;
    // Feedback publish
    cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Master joint states subscriber
    master_sub = n.subscribe<sensor_msgs::JointState>("/omni1_joint_states", 1, &SlaveController::masterJointsCallback, this);

    // Slave pose and velocity subscriber
    slave_sub = n.subscribe("/ground_truth/state", 1, &SlaveController::slaveOdometryCallback, this);

    // slave_sub = n.subscribe("/pose", 1, &SlaveController::slaveOdometryCallback, this);
    // navedata            = n.subscribe("/ardrone/navdata" , 1, &SlaveController::get_navdata   , this);
    //std::cout << "velocities min " << slave_min_.transpose() << std::endl ;
    // std::cout << "velocities max " << slave_max_.transpose() << std::endl ;


    force_feedback_sub  = n_.subscribe("pf_force_feedback" , 1, &SlaveController::getforce_feedback   , this);
    //velocity_cmd_sub = n.subscribe("R/cmd_vel",1,&SlaveController::getvel_feedback, this );

    std::cout << "end of constructor" << std::endl ;

}

//void SlaveController::getvel_feedback(const geometry_msgs::Twist::ConstPtr & vel)
//{
//  velocity << vel->linear.x , 0 , 0 ,0 , 0, 0 ,
//              0 , vel->linear.y,0,0,0,0,
//              0,0,vel->linear.z,0,0,0,
//              0,0,0,0,0,0,
//              0,0,0,0,0,vel->angular.z;

//}
void SlaveController::getforce_feedback(const geometry_msgs::PoseStamped::ConstPtr & force)
{

    force_stop <<  force->pose.position.x,
            force->pose.position.y,
            force->pose.position.z,
            0,
            0,
            0;
    std::cout << "force_x" << force->pose.position.x << std::endl ;
    std::cout << "force_y" << force->pose.position.y << std::endl ;
    std::cout << "force_z" << force->pose.position.z << std::endl ;
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
void SlaveController::get_navdata(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    battery_per   = msg->batteryPercent ;
    std::cout << "Baterry: " << battery_per << std::endl;

}
void SlaveController::paramsCallback(haptic_teleoperation::SlaveControllerConfig &config, uint32_t level)
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
    std::cout << "getting joint data " << std::endl;
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


    current_velocity_master=(current_pose_master-previous_pose_master)/period;

    //double yaw_master_scaled=yaw_master*master_to_slave_scale(5,0);

    /////////////////////////
    // Scale to slave side //
    /////////////////////////

    // x_m, y_m, z_m maps to velocities in slave side
    current_pose_master_scaled(0,0)=(current_pose_master(0,0)-master_min(0,0))*master_pose_slave_velocity_scale(0,0)+slave_velocity_min(0,0);

    std::cout << "current_pose_master_scaled x " << current_pose_master_scaled(0,0) ;

    current_pose_master_scaled(1,0)=(current_pose_master(1,0)-master_min(1,0))*master_pose_slave_velocity_scale(1,0)+slave_velocity_min(1,0);
    current_pose_master_scaled(2,0)=(current_pose_master(2,0)-master_min(2,0))*master_pose_slave_velocity_scale(2,0)+slave_velocity_min(2,0);
    // relative angular position changes in master side maps to relative angular position changes in slave side
    current_pose_master_scaled(3,0)=(current_pose_master(3,0))*master_to_slave_scale(3,0);
    current_pose_master_scaled(4,0)=(current_pose_master(4,0))*master_to_slave_scale(4,0);
    current_pose_master_scaled(5,0)=(current_pose_master(5,0))*master_to_slave_scale(5,0);


    // Velocity master
    current_velocity_master_scaled=(current_pose_master_scaled-previous_pose_master_scaled)/period;
    std::cout << "current_velocity_master_scaled  " << current_velocity_master_scaled ;

    master_new_readings=true;
    feedback();
    previous_pose_master=current_pose_master;
    previous_pose_master_scaled=current_pose_master_scaled;
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

    if(!init_slave_readings)
    {
        previous_pose_slave << msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z,
                roll-previous_pose_slave(3,0),
                pitch-previous_pose_slave(4,0),
                yaw; // should be relative
        //std::cout << "yaw:" << yaw << " yaw previous:" << yaw_slave_previous << std::endl;

        yaw_slave_previous=yaw;
        init_slave_readings=true;
        return;
    }
    else
    {
        // lastPositionUpdate      = ros::Time::now().toSec();

        current_pose_slave << msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z,
                roll-previous_pose_slave(3,0),
                pitch-previous_pose_slave(4,0),
                yaw-yaw_slave_previous; // should be relative
        //std::cout << "yaw:" << yaw << " yaw previous:" << yaw_slave_previous << std::endl;

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

void SlaveController::feedback()
{
    geometry_msgs::Twist twist_msg;

    if (control_event && force_stop(0,0) > -1.0) //  && !lastPositionUpdate) &&  (battery_per > 30) //
    {
       // std::cout << "force_stop(1,0) in if " << force_stop(0,0) << std::endl ;
        //  std::cout << "velocities min " << slave_velocity_min.transpose() << std::endl ;
        //  std::cout << "velocities max " << slave_velocity_max.transpose() << std::endl ;
        //Eigen::Matrix<double,6,1> r=current_velocity_master_scaled+lambda*current_pose_master_scaled;
        Eigen::Matrix<double,6,1> r=current_pose_master_scaled;
        //Eigen::Matrix<double,6,6> feeback_matrix =
        //       (current_pose_master_scaled - current_pose_slave)* Kp.transpose() +
        //     (r - current_velocity_slave)                     * Kd.transpose() +
        //    (current_velocity_master_scaled  - current_velocity_slave) * Bd.transpose();

        //  Eigen::Matrix<double,6,6> feeback_matrix =  (r - current_velocity_slave)* Kd.transpose() ;// r * Kd.transpose() ;
        Eigen::Matrix<double,6,6> feeback_matrix =   r * Kd.transpose() ;


        // sending command velocities
        twist_msg.linear.x=2*feeback_matrix(0,0);
        twist_msg.linear.y=2*feeback_matrix(1,1);
        //if ( current_pose_slave (2,0) < 1.8 )
        twist_msg.linear.z=2*feeback_matrix(2,2);
        twist_msg.angular.z=2*feeback_matrix(5,5);
        master_new_readings=false;
        slave_new_readings=false;

    }
    else if (control_event && force_stop(0,0) < -1.0 && current_pose_master(0,0) > 0.15 )//&& current_pose_master(0,0) > 0.0 //
    {
        std::cout << "IF  else 1" <<  std::endl ;

        std::cout << "force_stop(1,0) in else" << force_stop(0,0) << std::endl ;
        std::cout << "current_pose_master(0,0)" <<current_pose_master(0,0) << std::endl ;
        std::cout << "current_pose_master(1,0)" <<current_pose_master(1,0) << std::endl ;
        twist_msg.linear.x=0.0;
        twist_msg.linear.y=0.0;
        twist_msg.linear.z=0.0;
        master_new_readings=false;
        slave_new_readings=false;

    }
    else if (control_event && force_stop(0,0) < -1.0 && current_pose_master(0,0) < 0.15 )
{
        std::cout << "IF  else 2" <<  std::endl ;


        std::cout << "current_pose_master(0,0)" <<current_pose_master(0,0) << std::endl ;
        std::cout << "current_pose_master(1,0)" <<current_pose_master(1,0) << std::endl ;

        Eigen::Matrix<double,6,1> r=current_pose_master_scaled;
        Eigen::Matrix<double,6,6> feeback_matrix =   r * Kd.transpose() ;



        twist_msg.linear.x=2*feeback_matrix(0,0);
        twist_msg.linear.y=2*feeback_matrix(1,1);
        twist_msg.linear.z=2*feeback_matrix(2,2);
        twist_msg.angular.z=2*feeback_matrix(5,5);

        master_new_readings=false;
        slave_new_readings=false;
    }
    else
    {
        std::cout << "JUST ELSE" <<  std::endl ;



    }


    cmd_pub.publish(twist_msg);
    std::cout << "bublished succes: " << std::endl;


}
