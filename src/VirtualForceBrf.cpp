
#include "haptic_teleoperation/VirtualForceBrf.h"

VirtualForceBrf::VirtualForceBrf(ros::NodeHandle & n_):ForceField(n_)
{std::cout << "child constructor" << std::endl ; }



void  VirtualForceBrf::setGain( double g)
{
   gain = g ;
}

double  VirtualForceBrf::getGain(){ return  gain; }

Eigen::Vector3d VirtualForceBrf::getForcePoint (Eigen::Vector3d & c_current , Eigen::Vector3d & robot_vel ) {

    robot_vel(0) = 1 ;
    robot_vel(1) = 0 ;
    robot_vel(2) = 0 ;
    double dstop ;
}





