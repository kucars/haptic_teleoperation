
#ifndef VIRTUAL_FORCE_BRF_
#define VIRTUAL_FORCE_BRF_

#include "haptic_teleoperation/ForceField.h"



class VirtualForceBrf : public ForceField
{
public:
    VirtualForceBrf(ros::NodeHandle & n_);
    VirtualForceBrf() {std::cout << "default child constructor" << std::endl ; }
    ~VirtualForceBrf(){}
    Eigen::Vector3d getForcePoint(geometry_msgs::Point32 & c_current, Eigen::Vector3d robot_velocity) ;
    void setGain(double gain);
    double getGain();
private: 
    double gain;

};
#endif
