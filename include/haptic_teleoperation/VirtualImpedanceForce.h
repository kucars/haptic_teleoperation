
#ifndef VIRTUAL_IMPEDANCE_FORCE_
#define VIRTUAL_IMPEDANCE_FORCE_

#include "haptic_teleoperation/ForceField.h"



class VirtualImpedanceForce : public ForceField
{
public:
    VirtualImpedanceForce(ros::NodeHandle & n_);
    VirtualImpedanceForce() {std::cout << "default child constructor" << std::endl ; }
    ~VirtualImpedanceForce(){}
    Eigen::Vector3d getForcePoint (geometry_msgs::Point32 & c_current , Eigen::Vector3d  robot_vel) ;


private: 


};
#endif
