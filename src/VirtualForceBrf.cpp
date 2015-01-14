
#include "haptic_teleoperation/VirtualForceBrf.h"

VirtualForceBrf::VirtualForceBrf(ros::NodeHandle & n_):ForceField(n_)
{std::cout << "child constructor" << std::endl ; }



void  VirtualForceBrf::setGain( double g)
{
   gain = g ;
}

double  VirtualForceBrf::getGain(){ return  gain; }

Eigen::Vector3d VirtualForceBrf::getForcePoint (Eigen::Vector3d & c_current , Eigen::Vector3d & robot_vel , double gain ) {

    robot_vel(0) = 1 ;
    robot_vel(1) = 0 ;
    robot_vel(2) = 0 ;
    double dstop ;

    float dstop = (v_i*v_i) / (2*a_max ) ;
    float dres ;
    if ( v_i <= 0 ) dres = d + dstop ;
    else dres = d - dstop ;

    if ( dres <= 0 || (1+v_i)/dres >= 1/gain)
        return  1.0;
    else if ( (1+v_i)/dres <= 0)
        return 0 ;
    else
    {
        //std::cout << "risk vector" << gain *(1+v_i)/ dres  << std::endl ;
        return gain *(1+v_i)/ dres;
    }



}





