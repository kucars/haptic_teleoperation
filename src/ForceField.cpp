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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <cvaux.h>
#include <sstream>
#include "haptic_teleoperation/ForceField.h"

Eigen::Vector3d resulting_force ;


ForceField::ForceField(ros::NodeHandle & n_):n(n_)
{
    init_flag = true;
    init_flag_pose = true;

    std::cout << "parent constructor" << std::endl;


    laser_sub = n_.subscribe("/scan",1, &ForceField::laserCallback, this);

<<<<<<< HEAD
   // slave_pose_sub = n_.subscribe("/mavros/vision_pose/pose" , 100 , &ForceField::poseCallback, this );
    slave_pose_sub = n_.subscribe("/ground_truth/state" , 100 , &ForceField::pose2Callback, this );


=======
    slave_pose_sub = n_.subscribe("/ground_truth/state" , 100 , &ForceField::poseCallback, this );
>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3
    //slave_pose_sub = n_.subscribe("/ground_truth/state" , 100 , &ForceField::poseCallback, this);
    //  slave_pose_sub = n_.subscribe("/RosAria/pose" , 100 , &ForceField::poseCallback, this);
    // slave_pose_sub = n_.subscribe("/Pioneer3AT/pose" , 100 , &ForceField::poseCallback, this); // GAZEBO

    laser_pub = n_.advertise<sensor_msgs::PointCloud>("pointCloudObs", 100);
    virtual_force_pub = n_.advertise<geometry_msgs::PoseStamped>("/virtual_force_feedback", 100);
    visualization_markers_pub = n_.advertise<visualization_msgs::MarkerArray>("risk_vector_marker", 1);

    std::cout << "test" << std::endl;

}

//void ForceField::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & robot_state)
//{
//    double preT, nowT ;
//    Eigen::Vector3d robotVel ;
//    // std::cout << "get robot velocity " << std::endl ;
//    if(init_flag_pose)
//    {
//        PreRobotPose(0) = robot_state->pose.position.x ;
//        PreRobotPose(1) = robot_state->pose.position.y ;
//        PreRobotPose(2) = robot_state->pose.position.z ;
//        preT = ros::Time::now().toSec() ;
//        init_flag_pose = false ;
//        std::cout << "Init_flag "   << std::endl ;
//        poseQ[0] = robot_state->pose.orientation.x;
//        poseQ[1] = robot_state->pose.orientation.y;
//        poseQ[2] = robot_state->pose.orientation.z;
//        poseQ[3] = robot_state->pose.orientation.w;
//        return ;
//    }

//    else
//    {
//        nowT = ros::Time::now().toSec() ;
//        CurrentRobotPose(0) = robot_state->pose.position.x ;
//        CurrentRobotPose(1) = robot_state->pose.position.y ;
//        CurrentRobotPose(2) = robot_state->pose.position.z ;

//        robotVel(0) =  CurrentRobotPose(0) - PreRobotPose(0) / (nowT - preT) ;
//        robotVel(1) =  CurrentRobotPose(1) - PreRobotPose(1)  / (nowT - preT)  ;
//        robotVel(2) =  CurrentRobotPose(2) - PreRobotPose(2) / (nowT - preT) ;

//        setRobotVelocity(robotVel) ;
//        PreRobotPose  = CurrentRobotPose ;
//        preT = ros::Time::now().toSec() ;

//        poseQ[0] = robot_state->pose.orientation.x;
//        poseQ[1] = robot_state->pose.orientation.y;
//        poseQ[2] = robot_state->pose.orientation.z;
//        poseQ[3] = robot_state->pose.orientation.w;



//    }
//    // std::cout << "qx: " << poseQ[0] << " qy: " <<poseQ[1] << std::endl ;
//    tf::Quaternion q(poseQ[0], poseQ[1] ,poseQ[2],poseQ[3]);
//    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
//    //std::cout << "Yaw from callback  " << yaw * 180 / PI << std::endl ;

//}
void ForceField::poseCallback(const nav_msgs::Odometry::ConstPtr & robot_state)
{
    double preT, nowT ;
    Eigen::Vector3d robotVel ;
    std::cout << "get robot velocity " << std::endl ;
    if(init_flag_pose)
    {
        PreRobotPose(0) = robot_state->pose.pose.position.x ;
        PreRobotPose(1) = robot_state->pose.pose.position.y ;
        PreRobotPose(2) = robot_state->pose.pose.position.z ;
        preT = ros::Time::now().toSec() ;
        init_flag_pose = false ;
        std::cout << "Init_flag "   << std::endl ;
        poseQ[0] = robot_state->pose.pose.orientation.x;
        poseQ[1] = robot_state->pose.pose.orientation.y;
        poseQ[2] = robot_state->pose.pose.orientation.z;
        poseQ[3] = robot_state->pose.pose.orientation.w;
        return ;
    }

    else
    {
        nowT = ros::Time::now().toSec() ;
        CurrentRobotPose(0) = robot_state->pose.pose.position.x ;
        CurrentRobotPose(1) = robot_state->pose.pose.position.y ;
        CurrentRobotPose(2) = robot_state->pose.pose.position.z ;

        robotVel(0) =  CurrentRobotPose(0) - PreRobotPose(0) / (nowT - preT) ;
        robotVel(1) =  CurrentRobotPose(1) - PreRobotPose(1)  / (nowT - preT)  ;
        robotVel(2) =  CurrentRobotPose(2) - PreRobotPose(2) / (nowT - preT) ;

        setRobotVelocity(robotVel) ;
        PreRobotPose  = CurrentRobotPose ;
        preT = ros::Time::now().toSec() ;

        poseQ[0] = robot_state->pose.pose.orientation.x;
        poseQ[1] = robot_state->pose.pose.orientation.y;
        poseQ[2] = robot_state->pose.pose.orientation.z;
        poseQ[3] = robot_state->pose.pose.orientation.w;



    }
    // std::cout << "qx: " << poseQ[0] << " qy: " <<poseQ[1] << std::endl ;
    tf::Quaternion q(poseQ[0], poseQ[1] ,poseQ[2],poseQ[3]);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    //std::cout << "Yaw from callback  " << yaw * 180 / PI << std::endl ;
    std::cout << "CurrentRobotPose(0) from Call back   " << CurrentRobotPose(0) <<  std::endl ;
    std::cout << "CurrentRobotPose(1)  from Call back " << CurrentRobotPose(1) <<  std::endl ;
    std::cout << "CurrentRobotPose(2)   from Call back" << CurrentRobotPose(2) <<  std::endl ;


}




void ForceField::pose2Callback(const nav_msgs::Odometry::ConstPtr & robot_state)
{
//    double preT, nowT ;
    Eigen::Vector3d robotVel ;
//    // std::cout << "get robot velocity " << std::endl ;
//    if(init_flag_pose)
//    {
//        PreRobotPose(0) = robot_state->pose.position.x ;
//        PreRobotPose(1) = robot_state->pose.position.y ;
//        PreRobotPose(2) = robot_state->pose.position.z ;
//        preT = ros::Time::now().toSec() ;
//        init_flag_pose = false ;
//        std::cout << "Init_flag "   << std::endl ;
//        poseQ[0] = robot_state->pose.orientation.x;
//        poseQ[1] = robot_state->pose.orientation.y;
//        poseQ[2] = robot_state->pose.orientation.z;
//        poseQ[3] = robot_state->pose.orientation.w;
//        return ;
//    }

//    else
//    {
//        nowT = ros::Time::now().toSec() ;
//        CurrentRobotPose(0) = robot_state->pose.position.x ;
//        CurrentRobotPose(1) = robot_state->pose.position.y ;
//        CurrentRobotPose(2) = robot_state->pose.position.z ;

        robotVel(0) =  robot_state->twist.twist.linear.x  ;
        robotVel(1) =   robot_state->twist.twist.linear.y ;
        robotVel(2) =   robot_state->twist.twist.linear.z ;


//        robotVel(0) =  CurrentRobotPose(0) - PreRobotPose(0) / (nowT - preT) ;
//        robotVel(1) =  CurrentRobotPose(1) - PreRobotPose(1)  / (nowT - preT)  ;
//        robotVel(2) =  CurrentRobotPose(2) - PreRobotPose(2) / (nowT - preT) ;

        setRobotVelocity(robotVel) ;
//        PreRobotPose  = CurrentRobotPose ;
//        preT = ros::Time::now().toSec() ;

//        poseQ[0] = robot_state->pose.orientation.x;
//        poseQ[1] = robot_state->pose.orientation.y;
//        poseQ[2] = robot_state->pose.orientation.z;
//        poseQ[3] = robot_state->pose.orientation.w;



    //}
    // std::cout << "qx: " << poseQ[0] << " qy: " <<poseQ[1] << std::endl ;
   // tf::Quaternion q(poseQ[0], poseQ[1] ,poseQ[2],poseQ[3]);
  //  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    //std::cout << "Yaw from callback  " << yaw * 180 / PI << std::endl ;

}
void ForceField::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

   // std::cout<<"lasercallback"<<"\n";
    //*****************************************************************************
    //  double distance ;
    // double maxRange=0;
    //  double rangeMin = scan_in->range_min ;
    //  double rangeMax = scan_in->range_max ;
    //  std::cout<<"RangeMin:"<< rangeMin <<"RangeMax" <<rangeMax<<"\n";
    //  for(int i=0;i<scan_in->ranges.size();i++)
    //  {
    //      distance = scan_in->ranges[i] ;
    //     if (distance>maxRange)
    //	{
    // 		maxRange=distance;
    //	}
    //  }
    //  std::cout<<"Max Range:"<<maxRange<<"\n";
    //****************************************************************************


    // Transformation
    if(!listener_.waitForTransform(scan_in->header.frame_id,
<<<<<<< HEAD
                                   "base_link",
=======
                                   "world",
>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3
                                   scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                                   ros::Duration(2.0))){
        std::cout << "RETURN" << std::endl ;
        return;
    }

    sensor_msgs::PointCloud cloud;
<<<<<<< HEAD
    projector_.transformLaserScanToPointCloud("base_link",*scan_in, cloud,listener_);

    cloud.header.frame_id = "base_link" ;
    std::cout << "cloud_size" << cloud.points.size() << std::endl;
    std::cout << "cloud_0 :" << cloud.points[0].x << std::endl;
=======
  //  projector_.transformLaserScanToPointCloud("uav/baselink_ENU",*scan_in, cloud,listener_);
    projector_.transformLaserScanToPointCloud("world",*scan_in, cloud,listener_);
    //cloud.header.frame_id = "uav/baselink_ENU" ;
    cloud.header.frame_id = "world" ;

   // std::cout << "cloud_size" << cloud.points.size() << std::endl;
   // std::cout << "cloud_0 :" << cloud.points[0].x << std::endl;
>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3
    cloud.header.stamp = ros::Time::now();
    laser_pub.publish(cloud) ;

   // std::cout << "laserscan data" << std::endl ;
    //  runTestObstacles(cloud) ;
    //  runTestSamplePrf(cloud) ;
    computeForceField(cloud);
   // std::cout << " OUT OF FUNCTION " << std::endl ;
    feedbackMaster();

}

void ForceField::computeForceField(sensor_msgs::PointCloud & obstacles_positions_current)
{
   // std::cout << "computeForceField" << std::endl ;
    std::vector<Eigen::Vector3d> force_field;
    std::vector<Eigen::Vector3d> oop;

    unsigned int aux_it = obstacles_positions_current.points.size();
 //   std::cout << "aux1" << aux_it <<  std::endl ;
    if (aux_it != 0 )
    {
        std::cout << "laser data" <<   std::endl ;
        int count = 0 ;
        resulting_force=Eigen::Vector3d(0.0,0.0,0.0);

        for(int i=0; i<aux_it; i=i++)
        {
           std::cout << "for 1" <<   std::endl ;

            double obsMag = sqrt(pow(obstacles_positions_current.points[i].x - CurrentRobotPose(0), 2) + pow(obstacles_positions_current.points[i].y - CurrentRobotPose(1) , 2) + pow(obstacles_positions_current.points[i].z - CurrentRobotPose(2) , 2)) ;
            std::cout << "CurrentRobotPose(0)  " << CurrentRobotPose(0) <<  std::endl ;
            std::cout << "CurrentRobotPose(1)  " << CurrentRobotPose(1) <<  std::endl ;
            std::cout << "CurrentRobotPose(2)  " << CurrentRobotPose(2) <<  std::endl ;

            //std::cout << "obstacles_positions_current.points[i].x  " << obstacles_positions_current.points[i].x<<  std::endl ;
           // std::cout << "obstacles_positions_current.points[i].y  " << obstacles_positions_current.points[i].y <<  std::endl ;
           // std::cout << "obstacles_positions_current.points[i].z  " << obstacles_positions_current.points[i].z <<  std::endl ;


           std::cout << "ObsMag  " << obsMag <<  std::endl ;

            if(obsMag <= 3.0 && obsMag >= 0.5)
            {
                std::cout << "Start Calculation" <<   std::endl ;

                Eigen::Vector3d f = this->getForcePoint(obstacles_positions_current.points[i], getRobotVelocity()) ;
                Eigen::Vector3d dis ;
                dis(0) = obstacles_positions_current.points[i].x ;
                dis(1) = obstacles_positions_current.points[i].y ;
                dis(2) = obstacles_positions_current.points[i].z ;
                force_field.push_back(f);
                oop.push_back(dis) ;
                count++;
                resulting_force(0)=resulting_force(0) + f(0) ;
                resulting_force(1)=resulting_force(1) + f(1) ;
                resulting_force(2)=resulting_force(2) + f(2) ;

            }
            else{

               // std::cout << "out of range *****************************" << std::endl ;
                continue ;
            }

        }
        if (count == 0 )
        {
            //std::cout << "  1"  <<std::endl ;
            resulting_force=Eigen::Vector3d(0.0,0.0,0.0);

        }
        else
        {
          //  std::cout << "2: "  <<std::endl ;
            resulting_force(0) = resulting_force(0)/count ;
            resulting_force(1) = resulting_force(1)/count ;
            resulting_force(2) = resulting_force(2)/count ;
            count = 0.0;
        }
    }
    else
    {
        //std::cout << " 3: "  <<std::endl ;

        resulting_force=Eigen::Vector3d(0.0,0.0,0.0);
        std::cout << "zero force no obstacles " << std::endl ;
    }

    geometry_msgs::Point32 point;
    //Eigen::Vector3d point ( 0, 0 ,0) ;
    point.x = CurrentRobotPose(0) ; point.y = CurrentRobotPose(1) ; point.z = CurrentRobotPose(2);
    visualization_msgs::MarkerArray marker_array= rviz_arrows(force_field, oop, std::string("potential_field"));
    visualization_msgs::Marker marker=rviz_arrow(-resulting_force, point, 10000, std::string("resulting_risk_vector"));
    marker_array.markers.push_back(marker);
    visualization_markers_pub.publish(marker_array);
}

void ForceField::feedbackMaster()
{

  //  std::cout << " in master function  " <<  std::endl ;

    //std::cout << " frx " << resulting_force(0) << std::endl ;
    //std::cout << " fry " << resulting_force(1) << std::endl ;
    //std::cout << " frz " << resulting_force(2) << std::endl ;
    std::cout << "FORCE X: " <<  resulting_force(0) << " MSG X " << msg.pose.position.x << std::endl;
    msg.pose.position.x=resulting_force(0) ;
    msg.pose.position.y=resulting_force(1) ;
    msg.pose.position.z=resulting_force(2) ;
    msg.header.frame_id = "base_link" ;
    msg.header.stamp =  ros::Time::now();
    virtual_force_pub.publish(msg);
}

Eigen::Vector3d ForceField::getForcePoint(geometry_msgs::Point32  & c_current, Eigen::Vector3d  robot_velocity)
{
    std::cout << "getting forces parent" << std::endl ;

}



// ********************************* visualization **************************************************************
//visualization_msgs::MarkerArray ForceField::rviz_arrows(const std::vector<Eigen::Vector3d> & arrows, const sensor_msgs::PointCloud arrows_origins, std::string name_space)
//{
//    visualization_msgs::MarkerArray marker_array;
//    for(int i=0; i< arrows.size();i=i+30)
//    {
//        marker_array.markers.push_back(rviz_arrow(arrows[i], arrows_origins.points[i], (i+1), name_space));

//    }
//    return marker_array;
//}


visualization_msgs::MarkerArray ForceField::rviz_arrows(const std::vector<Eigen::Vector3d> & arrows, const std::vector<Eigen::Vector3d> & arrows_origins, std::string name_space)
{
    visualization_msgs::MarkerArray marker_array;
    for(int i=0; i< arrows.size();i=i+5)
    {
        geometry_msgs::Point32 a;
        a.x = arrows_origins[i].x() ;
        a.y = arrows_origins[i].y();
        a.z = arrows_origins[i].z();

        marker_array.markers.push_back(rviz_arrow(arrows[i], a , (i+1), name_space));

    }
    return marker_array;
}

visualization_msgs::Marker ForceField::rviz_arrow(const Eigen::Vector3d & arrow, const  geometry_msgs::Point32 & arrow_origin , int id, std::string name_space )
{

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
    // marker.header.frame_id = "/Pioneer3AT/laserscan"; // for pioneer
    //marker.header.frame_id = "laser";
<<<<<<< HEAD
    marker.header.frame_id = "laser0_frame";
=======
    marker.header.frame_id = "world";

//    marker.header.frame_id = "laser0_frame";
>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3
    //marker.header.stamp = ros::Time::now();
    marker.id = id;
    if(id==10000)
    {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.ns = name_space;      
        marker.scale.x =  arrow.norm() ; //arrow.x(); //norm();
        marker.scale.y = 0.1 ; //arrow.y();//0.1;//    marker.pose.orientation.x = rotation.x();
        //    marker.pose.orientation.y = rotation.y();
        //    marker.pose.orientation.z = rotation.z();
        //    marker.pose.orientation.w = rotation.w();

        marker.scale.z = 0.1;
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
    marker.pose.position.x = arrow_origin.x;
    marker.pose.position.y = arrow_origin.y;
    marker.pose.position.z = arrow_origin.z;

    marker.pose.orientation.x = rotation.x();
    marker.pose.orientation.y = rotation.y();
    marker.pose.orientation.z = rotation.z();
    marker.pose.orientation.w = rotation.w();

    if(arrow.norm()<0.0001 )
    {
        marker.scale.x = 0.001;
        marker.scale.y = 0.001;
        marker.scale.z = 0.001;
    }
    else if ( id != 10000)
    {
        marker.scale.x =  arrow.norm() ; //arrow.x(); //norm();
        marker.scale.y = 0.005 ; //arrow.y();//0.1;//    marker.pose.orientation.x = rotation.x();
        //    marker.pose.orientation.y = rotation.y();
        //    marker.pose.orientation.z = rotation.z();
        //    marker.pose.orientation.w = rotation.w();

        marker.scale.z = 0.1;
    }
    marker.color.a = 1.0;
    ros::Duration d(0.1);
    marker.lifetime = d ;

    marker.header.stamp = ros::Time::now() ;
    return marker;
}
// ***************************************** End visualization_msgs *******************************************

// **************************** run tests for PRF ****************************************************
void ForceField::runTestPrf(std::string testName)
{
    Eigen::Vector3d f;
    double laserRange = 4; // 4 meters
    double laserResolution = 0.005; // 5mm
    int numberOfPixels = int(2*laserRange/laserResolution);
    double obstX,obstY;
    geometry_msgs::Point32 currentPose;

    // Initialize image container with all black
    cv::Mat img(numberOfPixels,numberOfPixels, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat image = img;
    double maxF = 0;
    double minF = 1000000;
    for(int x=0;x<img.cols;x++)
    {
        for(int y=0;y<img.rows;y++)
        {
            obstX = (x - img.cols/2.0)*laserResolution;
            obstY = (img.rows/2.0 - y)*laserResolution;
            currentPose.x = obstX;
            currentPose.y = obstY;
            currentPose.z = 0;
            f = this->getForcePoint(currentPose,getRobotVelocity());
            double F = sqrt(f(0)*f(0) + f(1)*f(1) + f(2)*f(2));
            //            double Fn = f.norm() ;
            //            if ( f.norm() >0.99 )
            //            std::cout << "Fnorm:"<< Fn << "       F magn: " << F << std::endl;
            if(F>maxF)
                maxF = F;
            if(F<minF)
                minF = F;
            // This is how you get a pixel
            Vec3b color = image.at<cv::Vec3b>(cv::Point(x,y));
            color.val[0] = uchar(F * 255.0);
            color.val[1] = uchar(F * 255.0);
            color.val[2] =  uchar(F * 255.0);//(abs(F) * 255);
            //std::cout<<"F:"<<F<<" color:"<<color.val[0]<< "\n";
            // Set Pixel color to be Force indicative
            // Assuming that the force is normalzied between 0 and 1
            image.at<cv::Vec3b>(cv::Point(x,y)) = color;
        }
    }
    std::cout<<"PRF : Max f is:"<<maxF<<" min f:"<<minF<<"\n";
    imwrite(testName, img);
}
void ForceField::runTestSamplePrf(sensor_msgs::PointCloud &  array)
{
    Eigen::Vector3d f;
    double laserRange = 4.0; // 4 meters
    double laserResolution = 0.005; // 5mm
    int numberOfPixels = int(2*laserRange/laserResolution);
    double obstX,obstY;
    geometry_msgs::Point32 currentPose;

    // Initialize image container with all black
    cv::Mat img(numberOfPixels,numberOfPixels, CV_8UC3, cv::Scalar(255.0,255.0,255.0));
    cv::Mat image = img;
    double maxF = 0;
    double minF = 1000000;
    //  std::cout << "4- size" << array.points.size() <<std::endl;

    double maxX=0;
    double maxY=0;
    for(int i=0;i<array.points.size();i++)
    {
        obstX = array.points[i].x ;
        obstY = array.points[i].y;
        if (obstX>maxX)
        {
            maxX=obstX;
        }
        if (obstY>maxY)
        {
            maxY=obstY;
        }
    }
    std::cout<<"Max x:"<<maxX<<" maxY:"<<maxY<<"\n";
    for(int i=0;i<array.points.size();i++)
    {
        // std::cout << "i: " << i << "\n value" << array.points[i].x <<std::endl;
        obstX = array.points[i].x ;
        obstY = array.points[i].y;
        if(obstX > 4.0 || obstY > 4 )
            continue;
        int x = (obstX/laserResolution) + img.cols/2.0 ;
        int y = img.rows/2.0 - (obstY/laserResolution) ;
        // std::cout << "x: " << x << std::endl ;
        // std::cout << "y: " << y << std::endl ;
        currentPose.x = obstX;
        currentPose.y = obstY;
        currentPose.z = 0;
        f = this->getForcePoint(currentPose,getRobotVelocity());
        double F = sqrt(f(0)*f(0) + f(1)*f(1) + f(2)*f(2));


        if(F>maxF)
            maxF = F;
        if(F<minF)
            minF = F;
        // This is how you get a pixel

        Vec3b color = image.at<cv::Vec3b>(cv::Point(x,y));
        color.val[0] = uchar(F);
        color.val[1] = uchar(F);
        color.val[2] =  uchar(F);//(abs(F) * 255);
        //std::cout<<"F:"<<F<<" color:"<<color.val[0]<< "\n";
        // Set Pixel color to be Force indicative
        // Assuming that the force is normalzied between 0 and 1
        //  std::cout << "FORCE MAG: " << F << std::endl ;
        image.at<cv::Vec3b>(cv::Point(x,y)) = color;
    }
    std::cout<<"Max f is:"<<maxF<<" min f:"<<minF<<"\n";

    imwrite(" Image.png", img);
    exit(1) ;
}
void ForceField::runTestObstacles(sensor_msgs::PointCloud &  array)
{

    double laserRange = 4.0; // 4 meters
    double laserResolution = 0.005; // 5mm
    int numberOfPixels = int(2*laserRange/laserResolution);
    double obstX,obstY;
    geometry_msgs::Point32 currentPose;

    // Initialize image container with all black
    cv::Mat img(numberOfPixels,numberOfPixels, CV_8UC3, cv::Scalar(255.0,255.0,255.0));
    cv::Mat image = img;


    for(int i=0;i<array.points.size();i++)
    {

        obstX = array.points[i].x ;
        obstY = array.points[i].y;
        if(obstX > 4.0 || obstY > 4 )
            continue;
        int x = (obstX/laserResolution) + img.cols/2.0 ;
        int y = img.rows/2.0 - (obstY/laserResolution) ;
        // This is how you get a pixel
        Vec3b color = image.at<cv::Vec3b>(cv::Point(x,y));

        color.val[0] = uchar(255.0);
        color.val[1] = uchar(0.0);
        color.val[2] = uchar(0.0);

        image.at<cv::Vec3b>(cv::Point(x,y)) = color;
    }
    imwrite("obstacles.png", img);

}
String ForceField::testName(double dmin, double amax , double rpz ,double tahead, int numberOfTest , double vel  )
{

    std::string result ;
    std::stringstream sstm;
    sstm << numberOfTest+1 << "-"
         << " Minimum Distance: "   << dmin
         << " Max Acce: "           << amax
         << " Tahead: "              << tahead
         << " Ppz: "                 << rpz
         << " velocity in x:"       << vel
         << " Image.png";
    result = sstm.str();
    return result ;

}
// ***************************************** End PRF  *******************************************

// **************************************** Virtual impedance run tests************************
void ForceField::runTestVirtualImpedance()
{
    std::cout<<"runTestVirtualImpedance"<<"\n";

    Eigen::Vector3d f;
    double laserRange = 4; // 4 meters
    double laserResolution = 0.005; // 5mm
    int numberOfPixels = int(2*laserRange/laserResolution);
    double obstX,obstY;
    geometry_msgs::Point32 currentPose;

    // Initialize image container with all black
    cv::Mat img(numberOfPixels,numberOfPixels, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat image = img;
    double maxF = 0;
    double minF = 1000000;

    for(int x=0;x<img.cols;x++)
    {
        std::cout << " loop 1" << std::endl ;
        for(int y=0;y<img.rows;y++)
        {
            std::cout << "loop2" << std::endl ;
            obstX = (x - img.cols/2.0)*laserResolution;
            obstY = (img.rows/2.0 - y)*laserResolution;
            currentPose.x = obstX;
            currentPose.y = obstY;
            currentPose.z = 0;
            f = this->getForcePoint(currentPose , getRobotVelocity());
            //            c_previous = currentPose ;
            double F = sqrt(f(0)*f(0) + f(1)*f(1) + f(2)*f(2));
            //            double Fn = f.norm() ;
            //            if ( f.norm() >0.99 )
            //            std::cout << "Fnorm:"<< Fn << "       F magn: " << F << std::endl;
            if(F>maxF)
                maxF = F;
            if(F<minF)
                minF = F;
            // This is how you get a pixel
            Vec3b color = image.at<cv::Vec3b>(cv::Point(x,y));
            color.val[0] = uchar(F * 255.0);
            color.val[1] = uchar(F * 255.0);
            color.val[2] =  uchar(F * 255.0);//(abs(F) * 255);
            //std::cout<<"F:"<<F<<" color:"<<color.val[0]<< "\n";
            // Set Pixel color to be Force indicative
            // Assuming that the force is normalzied between 0 and 1
            image.at<cv::Vec3b>(cv::Point(x,y)) = color;
        }
    }

    //  std::cout<<"Max f is:"<<maxF<<" min f:"<<minF<<"\n";
    imwrite("testName Virtual Impedance7.png", img);
    exit(1) ;
}

// ***************************************** End VI *******************************************

// ********************* BRF run test functions ***************************************
void ForceField::runTestBrf(string nameOfTest)
{
    //std::cout << "runTestBrf" << std::endl ;

    Eigen::Vector3d f;
    double laserRange = 4; // 4 meters
    double laserResolution = 0.005; // 5mm
    int numberOfPixels = int(2*laserRange/laserResolution);
    double obstX,obstY;
    geometry_msgs::Point32 currentPose;

    //    // Initialize image container with all black
    cv::Mat img(numberOfPixels,numberOfPixels, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat image = img;
    double maxF = 0;
    double minF = 1000000;
    for(int x=0;x<img.cols;x++)
    {
        //     std::cout << " FOR LOOP ! " << std::endl ;
        for(int y=0;y<img.rows;y++)
        {
            //           std::cout << " FOR LOOP 2 " << std::endl ;

            obstX = (x - img.cols/2.0)*laserResolution;
            obstY = (img.rows/2.0 - y)*laserResolution;
            currentPose.x = obstX;
            currentPose.y = obstY;
            currentPose.z = 0;
            f = this->getForcePoint(currentPose,getRobotVelocity() );
            double F = sqrt(f(0)*f(0) + f(1)*f(1) + f(2)*f(2));
            //    std::cout << " F = " << F << std::endl;
            //            //            double Fn = f.norm() ;
            //            //            if ( f.norm() >0.99 )
            //            //            std::cout << "Fnorm:"<< Fn << "       F magn: " << F << std::endl;
            if(F>maxF)
                maxF = F;
            if(F<minF)
                minF = F;
            //            // This is how you get a pixel
            Vec3b color = image.at<cv::Vec3b>(cv::Point(x,y));
            color.val[0] = uchar(F * 255.0);
            color.val[1] = uchar(F * 255.0);
            color.val[2] =  uchar(F * 255.0);//(abs(F) * 255);
            //std::cout<<"F:"<<F<<" color:"<<color.val[0]<< "\n";
            // Set Pixel color to be Force indicative
            // Assuming that the force is normalzied between 0 and 1
            image.at<cv::Vec3b>(cv::Point(x,y)) = color;
        }
    }
    std::cout<<"Max f is:"<<maxF<<" min f:"<<minF<<"\n";
    imwrite(nameOfTest, img);
}

String ForceField::testNameBRF(double gain , double j )
{

    std::string result ;
    std::stringstream sstm;
    sstm << "velocity" << j<< "  Gain "   << gain<< " Image.png";
    result = sstm.str();
    return result ;

}
// ***************************************** End BRF  *******************************************


