/***************************************************************************
* Copyright (C) 2014  by                                             *
* Reem Ashour, Khalifa University Robotics Institute KURI               *
* <reem.ashour@kustar.ac.ae>                                          *
*                                                                          *
*                                        *
* This program is free software; you can redistribute it and/or modify     *
* it under the terms of the GNU General Public License as published by     *
* the Free Software Foundation; either version 2 of the License, or        *
* (at your option) any later version.                        *
*                                        *
* This program is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of        *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            *
* GNU General Public License for more details.                    *
*                                        *
* You should have received a copy of the GNU General Public License        *
* along with this program; if not, write to the                *
* Free Software Foundation, Inc.,                        *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA.            *
***************************************************************************/
#ifndef FORCEFIELD_ 
#define FORCEFIELD_ 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <cmath>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvaux.h>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

using namespace cv ;

const double PI=3.14159265359;
class ForceField
{
public:

    ros::Subscriber laser_sub;
    ros::Subscriber slave_pose_sub;
    ros::Publisher virtual_force_pub;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;

    // params
    bool init_flag ;
    std::vector<Eigen::Vector3d> obstacles_positions_current;
    std::vector<Eigen::Vector3d> obstacles_positions_previous;
    Eigen::Vector3d resulting_force;
    geometry_msgs::PoseStamped msg ;
    // constructor & destructor
    ForceField() {std::cout << "default parent constructor" << std::endl;}
    ~ForceField() {}
    ForceField(ros::NodeHandle & n_);
    // functions
    void computeForceField() ;
    void poseCallback(const nav_msgs::Odometry::ConstPtr & robot_velocity) ;
//    void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg); // I may use this one
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan) ;
    void feedbackMaster() ;
    void runTestPrf(string namOftest);
    void runTestSamplePrf(std::vector<Eigen::Vector3d> array) ;

    virtual Eigen::Vector3d getForcePoint(Eigen::Vector3d & c_current, Eigen::Vector3d robot_velocity) ;
    String testName(double dmin, double amax , double rpz ,double tahead, int numberOftest ,  double vel );
    void setRobotVelocity(Eigen::Vector3d robotVel)
    {
            robotVelocity(0) = robotVel(0) ;
            robotVelocity(1) = robotVel(1) ;
            robotVelocity(2) = robotVel(2) ;

    }

    Eigen::Vector3d getRobotVelocity()
    {
        return robotVelocity ;
    }

protected:
    ros::NodeHandle n;
    ros::NodeHandle n_priv;
    Eigen::Vector3d robotVelocity ;

};

#endif 
