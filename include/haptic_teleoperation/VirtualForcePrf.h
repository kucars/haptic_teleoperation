
#ifndef VIRTUAL_FORCE_PRF_
#define VIRTUAL_FORCE_PRF_

#include "haptic_teleoperation/ForceField.h"



class VirtualForcePrf : public ForceField
{
public:
    VirtualForcePrf(ros::NodeHandle & n_);
    VirtualForcePrf() {std::cout << "default child constructor" << std::endl ; }
    ~VirtualForcePrf(){}
    Eigen::Vector3d getForcePoint(geometry_msgs::Point32  & c_current , Eigen::Vector3d  robot_vel) ;
    void setMinDistance(double dmin);
    void setMaxAcceleration( double amax);
    void setCriticalAreaRadius(double rpz);
    void setTimeAhead(double tahead);
    double getMinDistance();
    double getMaxAcceleration();
    double getCriticalAreaRadius();
    double getTimeAhead();
    void setParameters(double dmin, double amax , double rpz ,double tahead  ) ;

private: 
    double minDistance;
    double maxAcceleration ;
    double criticalAreaRadius  ;
    double timeAhead  ;



};
#endif
