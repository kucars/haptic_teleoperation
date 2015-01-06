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
#include <cvaux.h>
#include <sstream>
#include "haptic_teleoperation/ForceField.h"


ForceField::ForceField(ros::NodeHandle & n_):n(n_)
{
    init_flag = false;
    std::cout << "parent constructor" << std::endl;
    virtual_force_pub = n_.advertise<geometry_msgs::PoseStamped>("virtual_force_feedback", 100);
     laser_sub = n_.subscribe("/scan", 100, &ForceField::laserCallback, this);
    //laser_sub = n_.subscribe("/cloud", 100, &ForceField::pointCloudCallback, this);
    slave_pose_sub = n_.subscribe("/RosAria/pose" , 100 , &ForceField::poseCallback, this );
}

void ForceField::poseCallback(const nav_msgs::Odometry::ConstPtr & robot_velocity)
{
    Eigen::Vector3d robotVel ;
    std::cout << "get robot velocity " << std::endl ;
    //    robotVel(0) = 0 ;
    //    robotVel(1) = 0 ;
    //    robotVel(2) = 0 ;
    robotVel(0) =  robot_velocity->twist.twist.linear.x ;
    robotVel(1) =  robot_velocity->twist.twist.linear.y  ;
    robotVel(2) =  robot_velocity->twist.twist.linear.z ;

    setRobotVelocity(robotVel) ;
}

//void ForceField::pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
//{
//    std::cout << "sonar callback start **** " << std::endl;

//    obstacles_positions_current.clear();
//    for(int i=0; i< msg->points.size(); ++i)
//    {
//        Eigen::Vector3d obstacle(msg->points[i].x,msg->points[i].y,msg->points[i].z);
//        obstacles_positions_current.push_back(obstacle);

//    }
//    computeForceField();
//    feedbackMaster();

//}
void ForceField::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

    std::cout << "laserCallBack " <<std::endl ;
    std::cout << "scan_in->header.frame_id" << std::endl ;


        if(!listener_.waitForTransform(scan_in->header.frame_id,
                                       "base_link",
                                       //ros::Time::now(),
                                       scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                                       ros::Duration(1.0))){
            std::cout << "return2" <<std::endl ;
            return;
        }

    std::cout << "point cloud  " <<std::endl ;

   sensor_msgs::PointCloud cloud;

   //  projector_.projectLaser(*scan_in, cloud);

    projector_.transformLaserScanToPointCloud("base_link",*scan_in, cloud,listener_);


    obstacles_positions_current.clear();

    for(int i=0; i< cloud.points.size(); ++i)
    {
        Eigen::Vector3d obstacle(cloud.points[i].x,cloud.points[i].y,0.0);
        obstacles_positions_current.push_back(obstacle);
    } // end of the for loop


    //    if(!init_flag)
    //    {
    //        std::cout << "initial data" << std::endl ;
    //        init_flag=true;
    //        obstacles_positions_previous=obstacles_positions_current;
    //        return;
    //    }

    computeForceField();
    std::cout << "system pause" <<std::endl ;
    feedbackMaster();
    //     std::system("pause") ;




}

void ForceField::computeForceField()
{
    std::cout << "computeForceField" << std::endl ;

    resulting_force=Eigen::Vector3d(0.0,0.0,0.0);
    std::vector<Eigen::Vector3d> force_field;
    unsigned int aux_it;
    // we take the smaller size to use it in the loop later
    // if(obstacles_positions_current.size()<=obstacles_positions_previous.size())
    aux_it=obstacles_positions_current.size();
    // else
    //   aux_it=obstacles_positions_previous.size();

    if (aux_it != 0 )
    {
        for(int i=0; i<aux_it; ++i)
        {
            Eigen::Vector3d f = this->getForcePoint(obstacles_positions_current[i], getRobotVelocity()) ;
            force_field.push_back(f);
            resulting_force+=force_field[i];
        }
        resulting_force = resulting_force/aux_it ;

    }
    else
    {
        resulting_force=Eigen::Vector3d(0.0,0.0,0.0);
        std::cout << "zero force no obstacles " << std::endl ;
    }
}

void ForceField::feedbackMaster()
{

    msg.pose.position.x=resulting_force.x() ;
    msg.pose.position.y=resulting_force.y() ;
    msg.pose.position.z=resulting_force.z() ;
    msg.header.frame_id = "base_link" ;
    msg.header.stamp =  ros::Time::now();

    virtual_force_pub.publish(msg);
}


Eigen::Vector3d ForceField::getForcePoint(Eigen::Vector3d & c_current, Eigen::Vector3d  robot_velocity)
{
    std::cout << "getting forces parent" << std::endl ;

}

void ForceField::runTestPrf(std::string testName)
{
    Eigen::Vector3d f;
    double laserRange = 4; // 4 meters
    double laserResolution = 0.005; // 5mm
    int numberOfPixels = int(2*laserRange/laserResolution);
    double obstX,obstY;
    Eigen::Vector3d currentPose;

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
            currentPose(0) = obstX;
            currentPose(1) = obstY;
            currentPose(2) = 0;
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
    std::cout<<"Max f is:"<<maxF<<" min f:"<<minF<<"\n";
    imwrite(testName, img);
}

void ForceField::runTestSamplePrf(std::vector<Eigen::Vector3d> array)
{
    Eigen::Vector3d f;
    double laserRange = 4; // 4 meters
    double laserResolution = 0.005; // 5mm
    int numberOfPixels = int(2*laserRange/laserResolution);
    double obstX,obstY;
    Eigen::Vector3d currentPose;

    // Initialize image container with all black
    cv::Mat img(numberOfPixels,numberOfPixels, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat image = img;
    double maxF = 0;
    double minF = 1000000;
    for(int i=0;i<array.size();i++)
    {
        obstX = array[i](1) ;
        obstY = array[i](2);
        currentPose(0) = obstX;
        currentPose(1) = obstY;
        currentPose(2) = 0;
        f = this->getForcePoint(currentPose,getRobotVelocity());
        double F = sqrt(f(0)*f(0) + f(1)*f(1) + f(2)*f(2));

        // This is how you get a pixel
        Vec3b color = image.at<cv::Vec3b>(cv::Point(obstX,obstY));
        color.val[0] = uchar(F * 255.0);
        color.val[1] = uchar(F * 255.0);
        color.val[2] =  uchar(F * 255.0);//(abs(F) * 255);
        //std::cout<<"F:"<<F<<" color:"<<color.val[0]<< "\n";
        // Set Pixel color to be Force indicative
        // Assuming that the force is normalzied between 0 and 1
        image.at<cv::Vec3b>(cv::Point(obstX,obstY)) = color;

    }
    imwrite("testname", img);
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

