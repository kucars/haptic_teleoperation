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
#include <geometry_msgs/Point32.h>

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
//#include <cvaux.h>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <visualization_msgs/MarkerArray.h>

using namespace cv ;

const double PI=3.14159265359;
class ForceField
{
public:



    // *********** PUb and SUb ***************** //

    ros::Subscriber laser_sub;
    ros::Subscriber slave_pose_sub;
    ros::Publisher virtual_force_pub;
    ros::Publisher laser_pub;
    ros::Publisher visualization_markers_pub ;

    // parameters, variables and msgs
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;


    Eigen::Vector3d resulting_force;
    Eigen::Vector3d pre_resulting_force;

    geometry_msgs::PoseStamped msg ;



    double roll, pitch , yaw ;

    double poseQ[4];
    Eigen::Vector3d PreRobotPose ;
    Eigen::Vector3d CurrentRobotPose ;


    //Eigen::Vector3d preResultingForce;
    // geometry_msgs::Point32 &  c_previous ;
    // params
    //  std::vector<Eigen::Vector3d> obstacles_positions_current;
    // std::vector<Eigen::Vector3d> obstacles_positions_previous;


    //************** constructor & destructor ****************** //
    ForceField() {std::cout << "default parent constructor" << std::endl;}
    ~ForceField() {}
    ForceField(ros::NodeHandle & n_);

    // ******************* Callback functions ************************* //
    void computeForceField(sensor_msgs::PointCloud & obstacles_positions_current) ;
    void feedbackMaster() ;

    // ************ helping functions ********************************** //
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan) ;
<<<<<<< HEAD
    void pose2Callback(const nav_msgs::Odometry::ConstPtr & robot_velocity) ;
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & robot_velocity) ;
=======
    void poseCallback(const nav_msgs::Odometry::ConstPtr & robot_velocity) ;
    //void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & robot_velocity) ;
>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3

    //    void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg); // I may use this one

    void visualization_fun(std::vector<Eigen::Vector3d> forces ,Eigen::Vector3d resulting_f , std::vector<Eigen::Vector3d> obstacles );

    // ******************* viviualization ****************
    //visualization_msgs::MarkerArray rviz_arrows(const std::vector<Eigen::Vector3d> & arrows, const sensor_msgs::PointCloud arrows_origins, std::string name_space);
    visualization_msgs::Marker rviz_arrow(const Eigen::Vector3d & arrow, const geometry_msgs::Point32 & arrow_origin, int id, std::string name_space ) ;
    visualization_msgs::MarkerArray rviz_arrows(const std::vector<Eigen::Vector3d> & arrows, const std::vector<Eigen::Vector3d> & arrows_origins, std::string name_space);
    //********* PRF run test functions ************** //
    void runTestPrf(string namOftest);
    String testName(double dmin, double amax , double rpz ,double tahead, int numberOftest ,  double vel );

    void runTestSamplePrf(sensor_msgs::PointCloud &  array) ;
    void runTestObstacles(sensor_msgs::PointCloud &  array) ;
    // ****** BRF run test functions ********************
    void runTestBrf(string namOftest) ;
    String testNameBRF(double gain, double x ) ;
    // ******* Virtual Impedance ********************
    void runTestVirtualImpedance() ;


    // virtual
    virtual Eigen::Vector3d getForcePoint(geometry_msgs::Point32 & c_current, Eigen::Vector3d robot_velocity) ;

    double distanse (double x , double y , double z)
    {
        sqrt(pow(x,2) + pow(y,2) + pow(z,2)) ;
    }

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

    bool init_flag ;
    bool init_flag_pose ;



};

#endif 
