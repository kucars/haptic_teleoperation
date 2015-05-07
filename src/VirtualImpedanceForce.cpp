
#include "haptic_teleoperation/VirtualImpedanceForce.h"

VirtualImpedanceForce::VirtualImpedanceForce(ros::NodeHandle & n_  , Eigen::Vector3d kp , Eigen::Vector3d kd):ForceField(n_)
{   kp_mat << kp.x(), 0, 0,
            0, kp.y(), 0,
            0, 0, kp.z();
    kd_mat << kd.x(), 0, 0,
            0, kd.y(), 0,
            0, 0, kd.z();


    std::cout << "child constructor" << std::endl ; }

Eigen::Vector3d VirtualImpedanceForce::getForcePoint (geometry_msgs::Point32 & c_current , Eigen::Vector3d  robot_vel) {

    // double p = sqrt(    ((c_current.x - CurrentRobotPose(0)) * (c_current.x - CurrentRobotPose(0)))
    //                                   + ((c_current.y - CurrentRobotPose(1)) * (c_current.y - CurrentRobotPose(1)))
    //                                   + ((c_current.z - CurrentRobotPose(2)) * (c_current.z - CurrentRobotPose(2))) ) ;



    double ro = 4.0 ;
    Eigen::Vector3d obsVector(0,0,0);
    //    obsVector(0) =  c_current.x * cos(yaw) - c_current.y * sin(yaw)  + CurrentRobotPose(0)  ;
    //    obsVector(1)=  c_current.x * sin(yaw) + c_current.y * cos(yaw) + CurrentRobotPose(1)  ;
    //    obsVector(2) = c_current.z + CurrentRobotPose(2) ;

    obsVector(0) = c_current.x  ;
    obsVector(1)=  c_current.y   ;
    obsVector(2) = c_current.z   ;


//        std::cout << "Obstacle position on x  " << obsVector(0)  << std::endl ;
//        std::cout << "Obstacle position on y  " << obsVector(1)  << std::endl ;
//        std::cout << "Obstacle position on z  " << obsVector(2)  << std::endl ;

    double obsMag = sqrt(pow(c_current.x, 2) + pow(c_current.y , 2) + pow(c_current.z , 2)) ;
    Eigen::Vector3d obsVecNorm = obsVector / obsMag ;

//    std::cout << "NORM on x  " << obsVecNorm(0)  << std::endl ;
//    std::cout << "NORM on y  " << obsVecNorm(1)  << std::endl ;
//    std::cout << "NORM on z  " << obsVecNorm(2)  << std::endl ;



    if (obsMag <0.5)
        std::cout << "obsMAG" << obsMag << std::endl ;
    std::cout << "obsMAG1 " << obsMag << std::endl ;


    // Eigen::Vector3d obsVecNorm = obsVector / p ;

    if(obsMag < ro) // < ro
    {

        std::cout << "obsMAGHHHHHHHHHHHHHHHHHHHHHHHHHH" << std::endl ;


        //  Spring Damper
        //  Eigen::Vector3d f=-((ro-obsMag)*obsVecNorm-(obsMag-c_previous.norm())*cobsVecNorm);
        //  Eigen::Vector3d f=-((ro-obsMag)*obsVecNorm-(obsMag-pre_resulting_force.norm())*obsVecNorm);
        //  Eigen::Vector3d f= kp_mat*((ro-obsMag)*obsVecNorm-kd_mat*(obsMag-pre_resulting_force.norm())*obsVecNorm);
        //  Spring
        //  Eigen::Vector3d f=kp_mat*(ro-c_current.norm())*c_current.normalized();
        //  Eigen::Vector3d f=-((ro-obsMag)*obsVecNorm);
        Eigen::Vector3d f=(-1 * kp_mat*((ro-obsMag)*obsVecNorm))/4;


        std::cout << "f on x" << f(0)  << std::endl ;
        std::cout << "f on y  " << f(1)  << std::endl ;
        std::cout << "f on z  " << f(2)  << std::endl ;


        //Eigen::Vector3d f=-1* kp_mat*(ro-p)*obsVecNorm;
        return f;
    }
    else
    {
        //std::cout << "force2" << std::endl ;
        return Eigen::Vector3d(0,0,0);
    }

}





