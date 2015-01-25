
#include "haptic_teleoperation/VirtualImpedanceForce.h"

VirtualImpedanceForce::VirtualImpedanceForce(ros::NodeHandle & n_):ForceField(n_)
{std::cout << "child constructor" << std::endl ; }

Eigen::Vector3d VirtualImpedanceForce::getForcePoint (geometry_msgs::Point32 & c_current , Eigen::Vector3d  robot_vel) {
    double ro = 4.0 ;
    Eigen::Vector3d obsVector;
    obsVector(0) = c_current.x ;
    obsVector(1)=  c_current.y ;
    obsVector(2) = c_current.z;
    double obsMag = sqrt(pow(c_current.x, 2) + pow(c_current.y , 2) + pow(c_current.z , 2)) ;
    Eigen::Vector3d obsVecNorm = obsVector / obsMag ;
    if(obsMag < ro) // < ro
    {

        // Spring Damper
      //  Eigen::Vector3d f=-((ro-obsMag)*obsVecNorm-(obsMag-c_previous.norm())*cobsVecNorm);
      //  std::cout << "force1" << std::endl ;
        // Spring
       // Eigen::Vector3d f=kp_mat*(ro-c_current.norm())*c_current.normalized();
        Eigen::Vector3d f= - (ro-obsMag)*obsVecNorm;
        return f;
    }
    else
    {
        //std::cout << "force2" << std::endl ;
        return Eigen::Vector3d(0,0,0);
    }


}





