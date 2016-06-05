/***************************************************************************
* Copyright (C) 2013 - 2014 by *
* Reem Ashour, Khalifa University Robotics Institute KURI *
* <reem.ashour@kustar.ac.ae> *
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
#include "haptic_teleoperation/SlaveController.h"
#include <time.h>
#include "ardrone_autonomy/Navdata.h"

#define RAD_TO_DEG 180/3.14
double battery_per ;
Eigen::Matrix<double,6,1> force_stop ;

bool inCollison ;

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
    // std::cout << " Initilization" << std::endl ;
    initParams();
    //slave_callback_type = boost::bind(&SlaveController::paramsCallback, this, _1, _2);
    //slave_server.setCallback(slave_callback_type);
    // Feedback publish
<<<<<<< HEAD

    //cmd_pub = n_.advertise<geometry_msgs::TwistStamped>("/uav/cmd_vel", 1);

=======
    //cmd_pub = n_.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3
    cmd_pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Master joint states subscriber

    master_sub = n_.subscribe<sensor_msgs::JointState>("/omni1_joint_states", 1, &SlaveController::masterJointsCallback, this);


    //force_feedback_sub = n_.subscribe<geometry_msgs::PoseStamped>("/virtual_force_feedback", 1, &SlaveController::feedbackFocreCallback, this);

    collision_flag = n_.subscribe<std_msgs::Bool>("/collision_flag" , 1, &SlaveController::get_inCollision , this);
    // Slave pose and velocity subscriber
<<<<<<< HEAD
    //slave_sub = n_.subscribe("/mavros/vision_pose/pose", 1, &SlaveController::slaveOdometryCallback, this);


    slave_sub = n_.subscribe("/ground_truth/state" , 1 , &SlaveController::slaveOdometryCallback, this);

=======
    slave_sub = n_.subscribe("/mavros/vision_pose/pose", 1, &SlaveController::slaveOdometryCallback, this);
>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3
    //force_feedback_sub = n_.subscribe("pf_force_feedback" , 1, &SlaveController::getforce_feedback , this);

}

void SlaveController::get_inCollision(const std_msgs::Bool::ConstPtr&  _inCollision)
{
    std::cout << "GET inCollison " << std::endl  ;
    inCollison = _inCollision->data ;
    std::cout << "inCollison" << inCollison << std::endl  ;
}


//void SlaveController::setfeedbackForce(Eigen::Vector3d &f)
//{
//    feedbackForce(0) = f(0) ;
//    feedbackForce(1) = f(1) ;
//    feedbackForce(2) = f(2) ;
//}

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
    n_priv.param<double>("slave_min_x", slave_min_x, 1.0);
    n_priv.param<double>("slave_min_y", slave_min_y, 1.0);
    n_priv.param<double>("slave_min_z", slave_min_z, 1.0);
    n_priv.param<double>("slave_min_roll", slave_min_roll, 1.0);
    n_priv.param<double>("slave_min_pitch", slave_min_pitch, 1.0);
    n_priv.param<double>("slave_min_yaw", slave_min_yaw, 1.0);
    Eigen::Matrix<double,6,1> slave_min;
    slave_min << slave_min_x,
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
    n_priv.param<double>("slave_max_x", slave_max_x, 1.0);
    n_priv.param<double>("slave_max_y", slave_max_y, 1.0);
    n_priv.param<double>("slave_max_z", slave_max_z, 1.0);
    n_priv.param<double>("slave_max_roll", slave_max_roll, 1.0);
    n_priv.param<double>("slave_max_pitch", slave_max_pitch, 1.0);
    n_priv.param<double>("slave_max_yaw", slave_max_yaw, 1.0);
    Eigen::Matrix<double,6,1> slave_max;
    slave_max << slave_max_x,
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
    n_priv.param<double>("master_min_x", master_min_x, 1.0);
    n_priv.param<double>("master_min_y", master_min_y, 1.0);
    n_priv.param<double>("master_min_z", master_min_z, 1.0);
    n_priv.param<double>("master_min_roll", master_min_roll, 1.0);
    n_priv.param<double>("master_min_pitch", master_min_pitch, 1.0);
    n_priv.param<double>("master_min_yaw", master_min_yaw, 1.0);
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
    n_priv.param<double>("master_max_x", master_max_x, 1.0);
    n_priv.param<double>("master_max_y", master_max_y, 1.0);
    n_priv.param<double>("master_max_z", master_max_z, 1.0);
    n_priv.param<double>("master_max_roll", master_max_roll, 1.0);
    n_priv.param<double>("master_max_pitch", master_max_pitch, 1.0);
    n_priv.param<double>("master_max_yaw", master_max_yaw, 1.0);
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

void SlaveController::paramsCallback(haptic_teleoperation::SlaveControllerConfig &config, uint32_t level)
{
    ROS_INFO_STREAM("Slave PID reconfigure Request ->" << " kp_x:" << config.kp_x
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
/* this function is used for autonomous collision avoidance **/
//void SlaveController::feedbackFocreCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
//{
//    double x = msg->pose.position.x ;
//    double y = msg->pose.position.y ;
//    double z = msg->pose.position.z ;
//    //std::cout << "X: " << x << " Y: " << y << " Z: " << z << std::endl ;
//    Eigen::Vector3d f(x,y,z) ;
//    setfeedbackForce(f) ;
//    //feedbackForce = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y ,msg->pose.position.z) ;
//}
// MASTER MEASUREMENTS
void SlaveController::masterJointsCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
    // std::cout << "getting joint data " << std::endl;
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
    // std::cout << "current_pose_master_scaled x " << current_pose_master_scaled(0,0) ;
    current_pose_master_scaled(1,0)=(current_pose_master(1,0)-master_min(1,0))*master_pose_slave_velocity_scale(1,0)+slave_velocity_min(1,0);
    current_pose_master_scaled(2,0)=(current_pose_master(2,0)-master_min(2,0))*master_pose_slave_velocity_scale(2,0)+slave_velocity_min(2,0);
    // relative angular position changes in master side maps to relative angular position changes in slave side
    current_pose_master_scaled(3,0)=(current_pose_master(3,0))*master_to_slave_scale(3,0);
    current_pose_master_scaled(4,0)=(current_pose_master(4,0))*master_to_slave_scale(4,0);
    current_pose_master_scaled(5,0)=(current_pose_master(5,0))*master_to_slave_scale(5,0);
    // Velocity master
    current_velocity_master_scaled=(current_pose_master_scaled-previous_pose_master_scaled)/period;
    // std::cout << "current_velocity_master_scaled " << current_velocity_master_scaled ;
    master_new_readings=true;
    feedback();
    previous_pose_master=current_pose_master;
    previous_pose_master_scaled=current_pose_master_scaled;
}

/* this function is used with mavros package where the type of msg for the position of the robot is PoseStammped */
// SLAVE MEASUREMENTS
//void SlaveController::slaveOdometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){


//    tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
//    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

//    std::cout << "YAW = " << yaw * 180 /3.4 << std::endl ;
//    //ROS_INFO("Pose Yaw:%f",pcl::rad2deg(yaw));

//    // Pose slave

//    //std::cout << " slaveOdometryCallback" << std::endl ;

//   // Eigen::Matrix<double,3,1> euler=Eigen::Quaterniond(msg->pose.orientation.w,
//    //                                                   msg->pose.orientation.x,
//     //                                                  msg->pose.orientation.y,
//      //                                                 msg->pose.orientation.z).matrix().eulerAngles(2, 1, 0);
//    //double yaw = euler(0,0);
//    //double pitch = euler(1,0);
//    //double roll = euler(2,0);

//    if(!init_slave_readings)
//    {

//        previous_pose_slave << msg->pose.position.x,
//                msg->pose.position.y,
//                msg->pose.position.z,
//                roll-previous_pose_slave(3,0),
//                pitch-previous_pose_slave(4,0),
//                yaw; // should be relative

//        yaw_slave_previous=yaw;
//        init_slave_readings=true;
//        return;
//    }
//    else
//    {
//        // lastPositionUpdate      = ros::Time::now().toSec();

//        current_pose_slave << msg->pose.position.x,
//                msg->pose.position.y,
//                msg->pose.position.z,
//                roll-previous_pose_slave(3,0),
//                pitch-previous_pose_slave(4,0),
//                yaw_slave_previous; // should be relative


//       // std::cout << "current_pose_slave(5,0)" << current_pose_slave(5,0) << std::endl ;;

//        //    double test = current_pose_slave(5,0) ;
//        //        std::cout << "yaw:" << yaw << "                  yaw previous:" << yaw_slave_previous << std::endl;
//        //        std::cout << "current_pose_slave:" << test  << "                 previous_pose_slave previous:" << previous_pose_slave(5,0) << std::endl;
//        //        std::cout << "yaw TO DEG:" << yaw*RAD_TO_DEG  << "                  yaw previous TO DEG:" << yaw_slave_previous * RAD_TO_DEG << std::endl;
//        //        std::cout << "current_pose_slave TO DEG (((:" << current_pose_slave(5,0)*RAD_TO_DEG  << "             previous_pose_slave previous TO DEG )))):" << previous_pose_slave(5,0) * RAD_TO_DEG << std::endl;

//        yaw_slave_previous=yaw;
//    }


//    slave_new_readings=true;
//    feedback();
//    previous_pose_slave=current_pose_slave;

//}

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
        // std::cout << "previous_pose_slave:" << previous_pose_slave(5,0) << " yaw:" << yaw << std::endl;
        // std::cout << "yaw:" << yaw << " yaw previous:" << yaw_slave_previous << std::endl;
        yaw_slave_previous=yaw;
        init_slave_readings=true;
        return;
    }
    else
    {
        // lastPositionUpdate = ros::Time::now().toSec();
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
<<<<<<< HEAD
=======

>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3
void SlaveController::feedback()
{
    geometry_msgs::TwistStamped twist_msg;

    geometry_msgs::Twist msg;
    /* the other condition has been used with the commercial quadrotor */
    if (control_event) // && force_stop(0,0) > -1.0 && !lastPositionUpdate) && (battery_per > 30) //
    {
        Eigen::Matrix<double,6,1> r=current_pose_master_scaled;
        Eigen::Matrix<double,6,6> feeback_matrix = r * Kd.transpose() ;
        /* this code has been used when creating a vertual fence from this file not from python file for testing using ground robot and simulation quadrotor*/
        //        double timeSample = 1 ; //0.05;
        //        double xBoundry = 9*0.6 ;
        //        double yBoundry = 5*0.6 ;
        //double forceNorm =sqrt(pow(feedbackForce(0,0),2) + pow(feedbackForce(0,1),2) + pow(feedbackForce(0,2),2)) ;
        //        if(geoFence(timeSample,current_pose_slave,feeback_matrix ,xBoundry , yBoundry) || inCollison)
        //        {
        //            std::cout << "OUSDIE ###################" << std:: endl ;
        //            twist_msg.linear.x=0.0;
        //            twist_msg.linear.y=0.0;
        //            twist_msg.linear.z=0.0;
        //            twist_msg.angular.z=feeback_matrix(5,5);
        //            master_new_readings=false;
        //            slave_new_readings=false;
        //        }
        //        else
        //        {
        double vx = feeback_matrix(0,0) ;
        double vy =  feeback_matrix(1,1)  ;
        double theta = current_pose_slave(5,0) ;

        /* this code has been used with the actual quadrotor and mavros packhgae */

        // msg.linear.x=(vx *cos(theta) )- (vy* sin(theta)) ;
        // msg.linear.y=(vx *sin(theta)) + (vy * cos(theta));
        // msg.linear.z= 0 ;
        if(inCollison)
        {
            msg.linear.x= 0 ;
            msg.linear.y= 0;
            msg.linear.z= 0 ;
            msg.angular.z=feeback_matrix(5,5);
        }
        else
        {
            msg.linear.x=vx ;
            msg.linear.y=vy;
            msg.linear.z= 0 ;
            msg.angular.z=feeback_matrix(5,5);

        }
        master_new_readings=false;
        slave_new_readings=false;
        //   }
    }
    twist_msg.twist = msg ;
    twist_msg.header.stamp = ros::Time::now() ;

    cmd_pub.publish(msg);
}

bool SlaveController::geoFence(double timeSample , Eigen::Matrix<double,6,1> currentPose , Eigen::Matrix<double,6,6> desiredVelocity , double xBoundry , double yBoundry )
{
    double theta = currentPose(5,0) ;
    double x = currentPose(0,0) ;
    double y = currentPose(1,0) ;
    double speedInx = desiredVelocity(0,0) ; // * sin(theta) ;
    double speedIny = desiredVelocity(1,1) ; //* cos(theta);
    double xPrime = speedInx*cos(theta)*timeSample ;
    double yPrime = speedInx*sin(theta)*timeSample ;

    if( (x+xPrime > xBoundry) || ( y+yPrime>yBoundry) ||( x+xPrime < 0.0) || ( y+yPrime< 0.0)  )
        return true ;
    else
        return false ;
}
