#include "haptic_teleoperation/VirtualImpedanceForce.h"

VirtualImpedanceForce::VirtualImpedanceForce(ros::NodeHandle & n_  , Eigen::Vector3d kp , Eigen::Vector3d kd):ForceField(n_)
{
    kp_mat << kp.x(), 0, 0,
            0, kp.y(), 0,
            0, 0, kp.z();
    kd_mat << kd.x(), 0, 0,
            0, kd.y(), 0,
            0, 0, kd.z();
    std::cout << "child constructor" << std::endl;
}

Eigen::Vector3d VirtualImpedanceForce::getForcePoint (geometry_msgs::Point32 & c_current , Eigen::Vector3d  robot_vel) {
    double ro = 3.0 ;
    Eigen::Vector3d obsVector(0,0,0);


//    obsVector(0) = c_current.x  ;
//    obsVector(1)=  c_current.y  ;
//    obsVector(2) = c_current.z  ;

    obsVector(0) = CurrentRobotPose(0) - c_current.x  ;
    obsVector(1)=  CurrentRobotPose(1) - c_current.y  ;
    obsVector(2) = CurrentRobotPose(2) - c_current.z  ;
    double obsMag = sqrt(pow(c_current.x - CurrentRobotPose(0), 2) + pow(c_current.y - CurrentRobotPose(1) , 2) + pow(c_current.z - CurrentRobotPose(2) , 2)) ;
    Eigen::Vector3d obsVecNorm = obsVector / obsMag ;

    if(obsMag < ro) // < ro
    {
        Eigen::Vector3d f=(-1 * kp_mat*((ro-obsMag)*obsVecNorm))/3;
        std::cout << "f on x" << f(0)  << std::endl ;
        std::cout << "f on y  " << f(1)  << std::endl ;
        std::cout << "f on z  " << f(2)  << std::endl ;
        return -f;
    }
    else
    {
        return Eigen::Vector3d(0,0,0);
    }

}





