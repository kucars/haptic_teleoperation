/***************************************************************************
* Copyright (C) 2014  by                                             *
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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>

#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <haptic_teleoperation/ForceFieldConfig.h>
#include <haptic_teleoperation/TwistArray.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/LaserScan.h>


const double PI=3.14159265359;
double lastTimeCalled ;//= ros::Time::now().toSec();


class ForceField
{
public:
    dynamic_reconfigure::Server<haptic_teleoperation::ForceFieldConfig> param_server;
    dynamic_reconfigure::Server<haptic_teleoperation::ForceFieldConfig>::CallbackType param_callback_type;
    ForceField(ros::NodeHandle & n_,
               double & freq_,
               double & ro_,
               Eigen::Vector3d kp_,
               Eigen::Vector3d kd_,
               double & laser_min_distance_,
               double & laser_max_distance_,
               double & robot_mass_,
               double & robot_radius_,
               std::string & pose_topic_name_,
               std::string & sonar_topic_name_):
        n(n_),
        freq(freq_),
        ro(ro_),
        kp(kp_),
        kd(kd_),
        laser_min_distance(laser_min_distance_),
        laser_max_distance(laser_max_distance_),
        robot_mass(robot_mass_),
        robot_radius(robot_radius_),
        pose_topic_name(pose_topic_name_),
        sonar_topic_name(sonar_topic_name_)

    {
        std::cout << "Constructor" << std::endl ;
        kp_mat << kp.x(), 0, 0,
                0, kp.y(), 0,
                0, 0, kp.z();
        kd_mat << kd.x(), 0, 0,
                0, kd.y(), 0,
                0, 0, kd.z();
        param_callback_type = boost::bind(&ForceField::paramsCallback, this, _1, _2);
        param_server.setCallback(param_callback_type);
        init_flag=false;

        obstacle_readings_sub = n_.subscribe("/scan", 100, &ForceField::laserCallback, this);
        visualization_markers_pub = n_.advertise<visualization_msgs::MarkerArray>("force_field_markers", 100);
        feedback_pub = n_.advertise<geometry_msgs::PoseStamped>("pf_force_feedback", 100); // increased the rate
        obstacle_pub = n_.advertise<geometry_msgs::PoseArray> ("obstacle", 1000) ;
        lastTimeCalled = ros::Time::now().toSec();
    }

    void computeForceField()
    {
        resulting_force=Eigen::Vector3d(0.0,0.0,0.0);
        // Compute current robot Velocity based on odometry readings
        std::vector<Eigen::Vector3d> force_field;
        unsigned int aux_it;
        // double Ve_Start = ros::Time::now().toSec();
        // we take the smaller size to use it in the loop later
        if(obstacles_positions_current.size()<=obstacles_positions_previous.size())
            aux_it=obstacles_positions_current.size();
        else
            aux_it=obstacles_positions_previous.size();

        if (aux_it != 0 )
        {
            for(int i=0; i<aux_it; ++i)
            {
                //       std::cout << "aux != 0  " << std::endl ;
                force_field.push_back(getForcePoint(obstacles_positions_current[i], obstacles_positions_previous[i], ro));
                resulting_force+=force_field[i];
            }
            resulting_force=resulting_force /aux_it;

        }
        else
        {
            resulting_force=Eigen::Vector3d(0.0,0.0,0.0);

            //   std::cout << "aux == 0  " << std::endl ;
        }// Publish visual markers to see in rviz

        visualization_msgs::MarkerArray marker_array=rviz_arrows(force_field, obstacles_positions_current, std::string("force_field"));
        visualization_msgs::Marker marker=rviz_arrow(resulting_force, Eigen::Vector3d(0,0,0), 0,   std::string("resulting_force"));
        marker_array.markers.push_back(marker);
        visualization_markers_pub.publish(marker_array);

    }

    Eigen::Vector3d getForcePoint(Eigen::Vector3d & c_current, Eigen::Vector3d & c_previous, const double & ro)
    {
        if(c_current.norm() < 0.8) // < ro
        {
            // Spring Damper
            Eigen::Vector3d f=kp_mat*(ro-c_current.norm())*c_current.normalized()
                    -kd_mat*(c_current.norm()-c_previous.norm())*c_current.normalized();

            // Spring
            //Eigen::Vector3d f=kp_mat*(ro-c_current.norm())*c_current.normalized();

            return f;
        }
        else
        {
            return Eigen::Vector3d(0,0,0);
        }
    }

private:
    // ROS
    ros::NodeHandle n;
    ros::Subscriber obstacle_readings_sub;
    ros::Publisher visualization_markers_pub;
    ros::Publisher feedback_pub ; // resulting force
    ros::Publisher obstacle_pub ;
    geometry_msgs::PoseStamped msg ;

    std::string pose_topic_name;
    std::string sonar_topic_name;
    std::string velocity_cmd_topic_name;
    double laser_min_distance;
    // Parameters
    double a_max;
    double ro;
    double freq;
    double robot_mass;
    double robot_radius;
    double laser_max_distance;

    Eigen::Vector3d kp, kd;
    Eigen::Matrix3d kp_mat, kd_mat;

    // Helper variables
    bool init_flag;

    std::vector<Eigen::Vector3d> obstacles_positions_current;
    std::vector<Eigen::Vector3d> obstacles_positions_previous;

    ros::Time previous_time;
    int count;

    Eigen::Vector3d resulting_force;


    void paramsCallback(haptic_teleoperation::ForceFieldConfig &config, uint32_t level)
    {
        ROS_DEBUG_STREAM("Force field reconfigure Request ->" << " kp_x:" << config.kp_x
                         << " kp_y:" << config.kp_y
                         << " kp_z:" << config.kp_z
                         << " kd_x:" << config.kd_x
                         << " kd_y:" << config.kd_y
                         << " kd_z:" << config.kd_z
                         << " ro:"   << config.ro);

        kp << config.kp_x,
                config.kp_y,
                config.kp_z;

        kd << config.kd_x,
                config.kd_y,
                config.kd_z;

        kp_mat << kp.x(), 0, 0,
                0, kp.y(), 0,
                0, 0, kp.z();
        kd_mat << kd.x(), 0, 0,
                0, kd.y(), 0,
                0, 0, kd.z();

        ro = config.ro;

        laser_max_distance = config.laser_max_distance;
        //slave_to_master_scale=Eigen::Matrix<double,3,1> (fabs(config.master_workspace_size.x/config.slave_workspace_size.x), fabs(config.master_workspace_size.y/config.slave_workspace_size.y), fabs(config.master_workspace_size.z/config.slave_workspace_size.z));
    }

    visualization_msgs::Marker rviz_arrow(const Eigen::Vector3d & arrow, const Eigen::Vector3d & arrow_origin, int id, std::string name_space )
    {

        //std::cout << "VISUALIZATION" << std::endl ;
        Eigen::Quaternion<double> rotation;
        if(arrow.norm()<0.0001)
        {
            rotation=Eigen::Quaternion<double>(1,0,0,0);
        }
        else
        {
            double rotation_angle=acos(arrow.normalized().dot(Eigen::Vector3d::UnitX()));
            Eigen::Vector3d rotation_axis=arrow.normalized().cross(Eigen::Vector3d::UnitX()).normalized();
            rotation=Eigen::AngleAxisd(-rotation_angle+PI,rotation_axis);
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = "laser"; // I have to change it to the base_link of the robot
        //marker.header.frame_id = "/Pioneer3AT/base_link";
        // marker.header.frame_id = "/RosAria/base_link";
        //   marker.header.frame_id = "base_link"; // I have to change it to the base_link of the robot

        marker.header.stamp = ros::Time();
        marker.id = id;
        if(id==0)
        {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.ns = name_space;
        }
        else
        {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.ns = name_space;
        }
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = arrow_origin.x();
        marker.pose.position.y = arrow_origin.y();
        marker.pose.position.z = arrow_origin.z();
        marker.pose.orientation.x = rotation.x();
        marker.pose.orientation.y = rotation.y();
        marker.pose.orientation.z = rotation.z();
        marker.pose.orientation.w = rotation.w();
        //std::cout <<"position:" <<marker.pose.position << std::endl;
        //std::cout <<"orientation:" <<marker.pose.orientation << std::endl;
        //marker.pose.orientation.x = 0;
        //marker.pose.orientation.y = 0;
        //marker.pose.orientation.z = 0;
        //marker.pose.orientation.w = 1;
        if(arrow.norm()<0.0001)
        {
            marker.scale.x = 0.001;
            marker.scale.y = 0.001;
            marker.scale.z = 0.001;
        }
        else
        {
            marker.scale.x = arrow.norm();
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
        }
        marker.color.a = 1.0;
        ros::Duration d(0.1);
        marker.lifetime = d ;
        marker.header.stamp = ros::Time::now();

        return marker;
    }

    visualization_msgs::MarkerArray rviz_arrows(const std::vector<Eigen::Vector3d> & arrows, const std::vector<Eigen::Vector3d> arrows_origins, std::string name_space)
    {

        visualization_msgs::MarkerArray marker_array;
        for(int i=0; i< arrows.size();++i)
        {
            marker_array.markers.push_back(rviz_arrow(arrows[i], arrows_origins[i], (i+1), name_space));
        }
        return marker_array;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
    {
        obstacles_positions_current.clear();
        float minAngle = laser_scan->angle_min ;
        float maxAngle = laser_scan->angle_max ;
        float range_min = laser_scan->range_min;
        float range_max = laser_scan->range_max;
        float incrementAngle = laser_scan->angle_increment ;
        float msgSize = laser_scan->ranges.size() ;

        // std::cout << "range min and max " << range_min << "   " << range_max << std::endl ;
        // std::cout << "\nminAngle" << minAngle << "\nmaxAngle" << maxAngle << "\nincreAngel" << incrementAngle << "\nmsgsize" << msgSize << std::endl ;

        float angle = minAngle;



        geometry_msgs::PoseArray obs_msg ;
        obs_msg.poses.clear();



        tf::TransformListener listener;
        listener.waitForTransform("laser", "base_link", ros::Time(0), ros::Duration(10.0));

        for (int i = 0; i < laser_scan->ranges.size();i++)
            // for(int i=0; i<msgSize; i++)
        {

            float range = laser_scan->ranges[i] ; // the distance from the min range angle to the max
            float angle  = laser_scan->angle_min + (i*laser_scan->angle_increment);



            geometry_msgs::Pose p;
            p.position.x =0.0 ;
            p.position.y =0.0 ;
            p.position.z =0.0 ;
            if(range <= 1.0 && range >= 0.5)
            {

                std::cout << "In the range" << std::endl ;
                float xPoint;
                float yPoint;
                float zPoint;

                geometry_msgs::PointStamped laser_point;

                laser_point.header.frame_id = "laser";
                laser_point.header.stamp = ros::Time();
                xPoint = laser_point.point.x = range*cos(angle) ;
                yPoint =  laser_point.point.y = range*sin(angle) ;
                zPoint =  laser_point.point.z = 0.0;

                Eigen::Vector3d obstacle(xPoint,yPoint,0.0);
                ROS_INFO_STREAM("INSIDE THE LIMITS: "<<obstacle.norm());
                obstacles_positions_current.push_back(obstacle);

                geometry_msgs::PointStamped odom_point;

                try{
                    listener.transformPoint("/base_link", laser_point, odom_point);
                    std::cout << "xPoint" << xPoint << std::endl ;
                    std::cout << "yPoint" << yPoint<< std::endl ;
                    std::cout << "obs_odom_x" << odom_point.point.x << std::endl ;
                    std::cout << "obsOdom_y" << odom_point.point.y << std::endl ;
                }
                catch(tf::TransformException& ex){
                    ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
                }

                //                float x_o = odom_point.point.x ;
                //                float y_o = odom_point.point.y ;

                p.position.x = odom_point.point.x ;;
                p.position.y = odom_point.point.y;
                std::cout << "obs_x: " <<  p.position.x  << std::endl ;
                std::cout << "obs_y: " << p.position.y << std::endl ;

                p.position.z = 0.0 ;
            }


            std::cout << "obs_x: " <<  p.position.x  << std::endl ;
            std::cout << "obs_y: " << p.position.y << std::endl ;
            std::cout << "obs_z: " << p.position.z << std::endl ;

            obs_msg.poses.push_back(p);


        } // end of the for loop
        obs_msg.header.frame_id = "/base_link";
        obs_msg.header.stamp = ros::Time::now();
        obstacle_pub.publish(obs_msg);








        if(!init_flag)
        {
            init_flag=true;
            obstacles_positions_previous=obstacles_positions_current;
            return;
        }

        computeForceField();
        feedbackMaster();
        obstacles_positions_previous=obstacles_positions_current;

    }




    void feedbackMaster()
    {


        double x = resulting_force.norm() ;
        double xmax = 0.8 ;
        double theta = atan(resulting_force.y()/resulting_force.x()) ;
        double force  ;
        if(x <= xmax)
            force = resulting_force.norm();
        else
        {
            if(resulting_force.norm() < 0 )
                force = -1 * xmax ;
            else if(resulting_force.norm() == 0 )
                force = 0 ;
            else
                force = 1 * xmax ;

        }

        msg.pose.position.x = -1 * force * cos(theta) ;
        msg.pose.position.y = -1 * force * sin(theta) ;

//         msg.pose.position.x=-resulting_force.x() ; // sign ??????  I used to add a nigative sing
//         msg.pose.position.y=-resulting_force.y() ;
        msg.pose.position.z=resulting_force.z() ;
        // std::cout << "resulting_risk_vector.x() " << msg.pose.position.x <<std::endl ;
        //  std::cout << "resulting_risk_vector.y() " << msg.pose.position.y <<std::endl ;
        //  std::cout << "resulting_risk_vector.z() " <<  msg.pose.position.z <<std::endl ;
        msg.header.stamp =  ros::Time::now();
        feedback_pub.publish(msg);
    }

};

int main(int argc, char **argv)
{
    std::cout << "MAIN" << std::endl ;

    /**
       * The ros::init() function needs to see argc and argv so that it can perform
       * any ROS arguments and name remapping that were provided at the command line. For programmatic
       * remappings you can use a different version of init() which takes remappings
       * directly, but for most command-line programs, passing argc and argv is the easiest
       * way to do it.  The third argument to init() is the name of the node.
       *
       * You must call one of the versions of ros::init() before using any other
       * part of the ROS system.
    */
    ros::init(argc, argv, "potential_field");
    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle n;

    ros::NodeHandle n_priv("~");

    // parameters
    double a_max;
    double ro;
    double freq;
    double robot_mass;
    double robot_radius;


    double kp_x;
    double kp_y;
    double kp_z;
    double kd_x;
    double kd_y;
    double kd_z;

    // Control gains
    n_priv.param<double>("Kp_x", kp_x, 1.0);
    n_priv.param<double>("Kp_y", kp_y, 1.0);
    n_priv.param<double>("Kp_z", kp_z, 1.0);
    n_priv.param<double>("Kp_x", kd_x, 10.0);
    n_priv.param<double>("Kp_y", kd_y, 10.0);
    n_priv.param<double>("Kp_z", kd_z, 10.0);

    Eigen::Vector3d kp(kp_x,kp_y,kp_z);
    Eigen::Vector3d kd(kd_x,kd_y,kd_z);


    double laser_max_distance;
    double laser_min_distance;


    //initialize operational parameters
    n_priv.param<double>("laser_max_distance", laser_max_distance,5.0);
    n_priv.param<double>("laser_min_distance", laser_min_distance,3.0);
    //initialize operational parameters
    n_priv.param<double>("ro", ro, 1.0);
    n_priv.param<double>("frequency", freq, 50.0);
    n_priv.param<double>("acc_max", a_max, 1.0);
    n_priv.param<double>("robot_mass", robot_mass, 1.0);
    n_priv.param<double>("robot_radius", robot_radius, 0.2);


    ros::Rate loop_rate(freq);




    std::string pose_topic_name = "/RosAria/pose" ;
    std::string sonar_topic_name = "/RosAria/sonar";
    ForceField potential_field(n, freq, ro, kp, kd, laser_min_distance, laser_max_distance, robot_mass, robot_radius, pose_topic_name, sonar_topic_name);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
