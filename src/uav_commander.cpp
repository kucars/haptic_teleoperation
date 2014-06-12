/***************************************************************************
* Copyright (C) 2013 - 2014 by                                                    *
* Tarek Taha and Rui P. de Figueiredo, Khalifa University Robotics Institute KURI *
*                     <tarek.taha@kustar.ac.ae>                            	  *
*                                                                          	  *
*                                                                           	  *
* This program is free software; you can redistribute it and/or modify     	  *
* it under the terms of the GNU General Public License as published by     	  *
* the Free Software Foundation; either version 2 of the License, or        	  *
* (at your option) any later version.                                      	  *
*                                                                          	  *
* This program is distributed in the hope that it will be useful,          	  *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           	  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             	  *
* GNU General Public License for more details.                              	  *
*                                                                          	  *
* You should have received a copy of the GNU General Public License        	  *
* along with this program; if not, write to the                            	  *
* Free Software Foundation, Inc.,                                          	  *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA.              	  *
***********************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "pctx_control/Control.h"
#include "visualeyez_tracker/TrackerPose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Eigen>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <haptic_teleoperation/PIDControllerConfig.h>
#include <sstream>
#include <string>
#include <sys/time.h>


#define PI 3.14159265
#define BILLION 1000000000

double x,y,z;

const double epsilon=0.1;

inline bool equalFloat(double a, double b, double epsilon)
{
    return fabs(a - b) < epsilon;
}

class PositionCommand
{
public:
    dynamic_reconfigure::Server<navigation::PIDControllerConfig> master_server;
    dynamic_reconfigure::Server<navigation::PIDControllerConfig>::CallbackType master_callback_type;

    int count;
    ros::Time previous_time;
    // PID gains
    Eigen::Matrix<double,6,1> Kp;
    Eigen::Matrix<double,6,1> Kd;
    Eigen::Matrix<double,6,1> Ki;

    Eigen::Matrix<double,6,1> previous_accum_error;

    std::string base_marker_id_;
    std::string head_marker_id_;
    bool got_pose_update_;
    bool goal_;
    bool got_base_marker_;
    bool got_head_marker_;

    Eigen::Vector3d base_marker_position;
    Eigen::Vector3d head_marker_position;

    Eigen::Matrix<double,6,1> previous_pose_;
    Eigen::Matrix<double,6,1> current_pose_;

    Eigen::Matrix<double,6,6> vel_cmd_previous;

    ros::Subscriber goal_sub;
    ros::Subscriber markers_sub;
    ros::Publisher vel_cmd;


    ros::NodeHandle n_;

    Eigen::Matrix<double,6,1> goal_pose_;


    PositionCommand(ros::NodeHandle & n, std::string & base_marker_id, std::string & head_marker_id,
                    Eigen::Matrix<double,6,1> & kp, Eigen::Matrix<double,6,1> & kd, Eigen::Matrix<double,6,1> & ki) :
        n_(n),
        base_marker_id_(base_marker_id),
        head_marker_id_(head_marker_id),
        Kp(kp),
        Kd(kd),
        Ki(ki),
        count(0),

        goal_(false)
    {
        ROS_INFO_STREAM("base marker id: "<<base_marker_id_);
        ROS_INFO_STREAM("head marker id: "<<head_marker_id_);
        master_callback_type = boost::bind(&PositionCommand::paramsCallback, this, _1, _2);
        master_server.setCallback(master_callback_type);

        //vel_cm_previous_vec=Eigen
        goal_sub = n_.subscribe("Goal", 1000, &PositionCommand::getGoal, this);

        markers_sub = n_.subscribe("/TrackerPosition", 1000, &PositionCommand::getUpdatedPose, this);
        //markers_sub = n_.subscribe("/nav_msgs/Odometry", 1000, &PositionCommand::getUpdatedPoseOdometry, this);
        vel_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    void paramsCallback(navigation::PIDControllerConfig &config, uint32_t level)
    {
        ROS_INFO_STREAM("PID reconfigure Request ->"
                        << " kp_x:" << config.kp_x
                        << " kp_y:" << config.kp_y
                        << " kp_z:" << config.kp_z
                        << " kp_x:" << config.kp_roll
                        << " kp_y:" << config.kd_pitch
                        << " kp_z:" << config.kd_yaw

                        << " kd_x:" << config.kd_x
                        << " kd_y:" << config.kd_y
                        << " kd_z:" << config.kd_z
                        << " kd_x:" << config.kd_roll
                        << " kd_y:" << config.kd_pitch
                        << " kd_z:" << config.kd_yaw


                        << " ki_x:" << config.ki_x
                        << " ki_y:" << config.ki_y
                        << " ki_z:" << config.ki_z
                        << " ki_x:" << config.ki_roll
                        << " ki_y:" << config.ki_pitch
                        << " ki_z:" << config.ki_yaw);

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

        Ki << config.ki_x,
                config.ki_y,
                config.ki_z,
                config.ki_roll,
                config.ki_pitch,
                config.ki_yaw;

        //slave_to_master_scale=Eigen::Matrix<double,3,1> (fabs(config.master_size.x/config.slave_size.x), fabs(config.master_size.y/config.slave_size.y), fabs(config.master_size.z/config.slave_size.z));
    }

private:

    void getUpdatedPose(const visualeyez_tracker::TrackerPose::ConstPtr& odom_orig);
    //void getUpdatedPoseOdometry(const nav_msgs::Odometry::ConstPtr& odom_orig);
    void getGoal(const geometry_msgs::PoseStamped::ConstPtr & goal);
};

/*void PositionCommand::getUpdatedPoseOdometry(const nav_msgs::Odometry::ConstPtr& odom_orig)
{
    //ROS_INFO(" Recieved Tracker Location: [%s] [%f] [%f] [%f]",trackerPose->tracker_id.c_str(),trackerPose->pose.x ,trackerPose->pose.y ,trackerPose->pose.z );
    geometry_msgs::Twist vel_cmd_msg;

    if(!goal_)
    {
        vel_cmd.publish(vel_cmd_msg);
    return;
    }

    if(count==0)
    {
    ROS_INFO("entrou");
        gettimeofday( &previous_time, NULL );
    ++count;
ros::Time current_time = ros::Time::now();
    previous_accum_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        return;
    }

    // Define a local reference frame
    Eigen::Vector3d uav_x=(head_marker_position-base_marker_position).normalized(); // Heading

    Eigen::Vector3d uav_y=(uav_x.cross(Eigen::Vector3d::UnitZ())).normalized();
    // LETS ASSUME THE UAV Z POINTS ALWAYS UP


    Eigen::Matrix<double, 3, 3> current_rotation_matrix;
    current_rotation_matrix << uav_x, uav_y, Eigen::Vector3d::UnitZ();
    Eigen::Matrix<double, 3, 1> current_euler = current_rotation_matrix.eulerAngles(2, 1, 0);

    double yaw = current_euler(0,0);
    double pitch = current_euler(1,0);
    double roll = current_euler(2,0);

        current_pose_ << odom_orig->pose.pose.position.x,
                     odom_orig->pose.pose.position.y,
             odom_orig->pose.pose.position.z,
             yaw,
             pitch,
             roll;

    std::cout << "current_pose: " << current_pose_ << std::endl;
    std::cout << "goal_pose: " << goal_pose_ << std::endl;

        Eigen::Matrix<double,6,1> error=goal_pose_-current_pose_;
        // Check if goal was reached

        if(error.norm()<epsilon) // CHANGE THIS
        {
            ROS_INFO("Reached goal!");
            goal_=false;
            count=0;
        }
        else //control
        {
        struct timeval current_time;
        gettimeofday( &previous_time, NULL );

                ROS_INFO_STREAM("position error norm: "<< error.transpose().norm());
        //timespec current_time;
            //clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);
        //double period = (double) current_time.tv_sec + (double) 1e-6 * tv.tv_usec;

        ros::Time current_time = ros::Time::now();
            //double period = (current_time.tv_sec - previous_time.tv_sec) + ((current_time.tv_usec - previous_time.tv_usec) / 1000000.0);
        double period=0.020;
        previous_time=current_time;
            //std::cout << "time diff: " << period << std::endl;

                //Eigen::Matrix<double,6,6> vel_cmd_current=Kp*(goal_pose_-current_pose_).transpose() + Kd*(current_pose_-previous_pose_).transpose()*freq_;
            Eigen::Matrix<double,6,1> vel_cm_previous_vec;
            vel_cm_previous_vec << vel_cmd_previous(0,0), vel_cmd_previous(1,1),vel_cmd_previous(2,2),vel_cmd_previous(3,3),vel_cmd_previous(4,4),vel_cmd_previous(5,5);
        Eigen::Matrix<double,6,1> current_error=(goal_pose_ - previous_pose_);
            Eigen::Matrix<double,6,1> vdiff = (current_error - (goal_pose_ - vel_cm_previous_vec));   // Differential Part
            Eigen::Matrix<double,6,1> current_accum_error = (current_error + previous_accum_error);   // Integral Part

                Eigen::Matrix<double,6,6> vel_cmd_current=Kp*(goal_pose_-current_pose_).transpose() + Kd*vdiff.transpose() + Ki*current_accum_error.transpose();

        std::cout << "vdiff: " << vdiff.transpose() << std::endl;
        std::cout << "error: " << (goal_pose_-current_pose_).transpose() << std::endl;
        std::cout << "vel_cm_current: "<< vel_cmd_current.transpose() << std::endl;
                vel_cmd_msg.linear.x=vel_cmd_current(0,0);
                vel_cmd_msg.linear.y=vel_cmd_current(1,1);
                vel_cmd_msg.linear.z=vel_cmd_current(2,2);
                //vel_cmd_msg.angular.x=vel_cmd_current(3,3);
                //vel_cmd_msg.angular.y=vel_cmd_current(4,4);
                //vel_cmd_msg.angular.z=vel_cmd_current(5,5);
                vel_cmd.publish(vel_cmd_msg);

        previous_accum_error=current_accum_error;
            vel_cmd_previous=vel_cmd_current;
        }

        previous_pose_=current_pose_;

}
*/

void PositionCommand::getUpdatedPose(const visualeyez_tracker::TrackerPose::ConstPtr& trackerPose)
{

    geometry_msgs::Twist vel_cmd_msg;

    if(!goal_)
    {
        vel_cmd.publish(vel_cmd_msg);
        return;
    }
    std::cout << trackerPose->tracker_id << " " << base_marker_id_<<  std::endl;
    //if (base_marker_id_.compare(std::string(trackerPose->tracker_id)))
    //if(trackerPose->tracker_id==base_marker_id_)
    unsigned pos = base_marker_id_.find("Channel");
    std::string strBase =base_marker_id_.substr(7,10);
    std::string strMsg =trackerPose->tracker_id.substr(7,10);
    int baseInt =atoi(strBase.c_str());
    int msgInt =atoi(strMsg.c_str());
    std::cout << baseInt << std::endl;
    std::cout << msgInt << std::endl;
    if (head_marker_id_.compare(std::string(trackerPose->tracker_id)) == 0)
   // if(baseInt==msgInt)
    {
        ROS_INFO_STREAM(" Head Tracker");
        head_marker_position=Eigen::Vector3d(trackerPose->pose.x/1000.0f, trackerPose->pose.y/1000.0f, trackerPose->pose.z/1000.0f);
        got_head_marker_=true;
    }
    else// if(trackerPose->tracker_id==head_marker_id_)
    {
        ROS_INFO_STREAM(" Base Tracker");
        base_marker_position=Eigen::Vector3d(trackerPose->pose.x/1000.0f, trackerPose->pose.y/1000.0f, trackerPose->pose.z/1000.0f);
        got_base_marker_=true;
    }

    // If both markers are available, do the control
    if(got_base_marker_ && got_head_marker_)
    {
        if(count==0)
        {

            previous_time = ros::Time::now();
            previous_accum_error << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            ++count;
            return;
        }

        // Define a local reference frame
        Eigen::Vector3d uav_x=(head_marker_position-base_marker_position).normalized(); // Heading

        Eigen::Vector3d uav_y=(uav_x.cross(Eigen::Vector3d::UnitZ())).normalized();
        // LETS ASSUME THE UAV Z POINTS ALWAYS UP


        Eigen::Matrix<double, 3, 3> current_rotation_matrix;
        current_rotation_matrix << uav_x, uav_y, Eigen::Vector3d::UnitZ();
        Eigen::Matrix<double, 3, 1> current_euler = current_rotation_matrix.eulerAngles(2, 1, 0);

        double yaw = current_euler(0,0);
        double pitch = current_euler(1,0);
        double roll = current_euler(2,0);

        current_pose_ << base_marker_position.x(),
                base_marker_position.y(),
                base_marker_position.z(),
                roll,
                pitch,
                yaw;

        std::cout << "current_pose: " << current_pose_ << std::endl;
        std::cout << "goal_pose: " << goal_pose_ << std::endl;

        Eigen::Matrix<double,6,1> error=goal_pose_-current_pose_;
        // Check if goal was reached
        if(error.norm()<epsilon) // CHANGE THIS
        {
            ROS_INFO("Reached goal!");
            goal_=false;
            count=0;
        }
        else //control
        {
            ros::Time current_time = ros::Time::now();

            double period=current_time.toSec()-previous_time.toSec();
            std::cout << "period:" << period << std::endl;
            previous_time=current_time;
            //std::cout << "time diff: " << period << std::endl;

            //Eigen::Matrix<double,6,6> vel_cmd_current=Kp*(goal_pose_-current_pose_).transpose() + Kd*(current_pose_-previous_pose_).transpose()*freq_;
            Eigen::Matrix<double,6,1> vel_cm_previous_vec;
            vel_cm_previous_vec << vel_cmd_previous(0,0), vel_cmd_previous(1,1),vel_cmd_previous(2,2),vel_cmd_previous(3,3),vel_cmd_previous(4,4),vel_cmd_previous(5,5);
            Eigen::Matrix<double,6,1> current_error=(goal_pose_ - previous_pose_);
            Eigen::Matrix<double,6,1> vdiff = (current_error - (goal_pose_ - vel_cm_previous_vec));   // Differential Part
            Eigen::Matrix<double,6,1> current_accum_error = (current_error + previous_accum_error);   // Integral Part

            Eigen::Matrix<double,6,6> vel_cmd_current=Kp*(goal_pose_-current_pose_).transpose() + Kd*vdiff.transpose() + Ki*current_accum_error.transpose();

            std::cout << "vdiff: " << vdiff.transpose() << std::endl;
            std::cout << "error: " << (goal_pose_-current_pose_).transpose() << std::endl;
            //std::cout << "vel_cm_current: "<< vel_cmd_current.transpose() << std::endl;
            vel_cmd_msg.linear.x=vel_cmd_current(0,0);
            vel_cmd_msg.linear.y=vel_cmd_current(1,1);
            vel_cmd_msg.linear.z=vel_cmd_current(2,2);
            //vel_cmd_msg.angular.x=vel_cmd_current(3,3);
            //vel_cmd_msg.angular.y=vel_cmd_current(4,4);
            //vel_cmd_msg.angular.z=vel_cmd_current(5,5);
            vel_cmd.publish(vel_cmd_msg);

            previous_accum_error=current_accum_error;
            vel_cmd_previous=vel_cmd_current;
        }

        previous_pose_=current_pose_;

    }
}

void PositionCommand::getGoal(const geometry_msgs::PoseStamped::ConstPtr & goal)
{
    ROS_INFO("GOT NEW GOAL");
    Eigen::Matrix<double,3,1> euler=Eigen::Quaterniond(goal->pose.orientation.w,
                                                       goal->pose.orientation.x,
                                                       goal->pose.orientation.y,
                                                       goal->pose.orientation.z).matrix().eulerAngles(2, 1, 0);
    double yaw = euler(0,0);
    double pitch = euler(1,0);
    double roll = euler(2,0);

    goal_pose_ << goal->pose.position.x,
            goal->pose.position.y,
            goal->pose.position.z,
            roll,
            pitch,
            yaw;
    goal_=true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_commander");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    std::string base_marker_id;
    std::string head_marker_id;
    double freq;

    n_priv.param<double>("freq", freq, 50.0);
    n_priv.param<std::string>("base_marker_id", base_marker_id, "teste");
    n_priv.param<std::string>("head_marker_id", head_marker_id, "teste2");

    ///////////
    // Gains //
    ///////////


    // Proportional (kd)
    double kp_x;
    double kp_y;
    double kp_z;
    double kp_roll;
    double kp_pitch;
    double kp_yaw;

    n_priv.param<double>("kp_x", kp_x, 0.5);
    n_priv.param<double>("kp_y", kp_y, 1.0);
    n_priv.param<double>("kp_z", kp_z, 1.0);
    n_priv.param<double>("kp_roll", kp_roll, 0.0);
    n_priv.param<double>("kp_pitch", kp_pitch, 0.0);
    n_priv.param<double>("kp_yaw", kp_yaw, 1.0);

    Eigen::Matrix<double,6,1> Kp;

    Kp << kp_x, kp_y, kp_z, kp_roll, kp_pitch, kp_yaw;

    // Derivative (kd)
    double kd_x;
    double kd_y;
    double kd_z;
    double kd_roll;
    double kd_pitch;
    double kd_yaw;

    n_priv.param<double>("kd_x", kd_x, 1.0);
    n_priv.param<double>("kd_y", kd_y, 1.0);
    n_priv.param<double>("kd_z", kd_z, 1.0);
    n_priv.param<double>("kd_roll", kd_roll, 0.0);
    n_priv.param<double>("kd_pitch", kd_pitch, 0.0);
    n_priv.param<double>("kd_yaw", kd_yaw, 1.0);

    Eigen::Matrix<double,6,1> Kd;

    Kd << kd_x,kd_y,kd_z, kd_roll, kd_pitch,kd_yaw;

    // Integral (ki)
    double ki_x;
    double ki_y;
    double ki_z;
    double ki_roll;
    double ki_pitch;
    double ki_yaw;

    n_priv.param<double>("ki_x", ki_x, 1.0);
    n_priv.param<double>("ki_y", ki_y, 1.0);
    n_priv.param<double>("ki_z", ki_z, 1.0);
    n_priv.param<double>("ki_roll", ki_roll, 0.0);
    n_priv.param<double>("ki_pitch", ki_pitch, 0.0);
    n_priv.param<double>("ki_yaw", ki_yaw, 1.0);

    Eigen::Matrix<double,6,1> Ki;

    Ki << ki_x, ki_y, ki_z, ki_roll, ki_pitch, ki_yaw;

    std::cout << "Kp gains:" << Kp << std::endl;
    std::cout << "Kd gains:" << Kd << std::endl;
    std::cout << "Ki gains:" << Ki << std::endl;
    PositionCommand position_commander(n, base_marker_id, head_marker_id, Kp, Kd, Ki);
    ros::Rate loop_rate(freq);
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
