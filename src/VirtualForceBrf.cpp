#include "haptic_teleoperation/VirtualForceBrf.h"
#include <Eigen/Eigen>

VirtualForceBrf::VirtualForceBrf(ros::NodeHandle & n_):ForceField(n_)
{std::cout << "child constructor" << std::endl ; }



void  VirtualForceBrf::setGain( double g)
{
   gain = g ;
}

double  VirtualForceBrf::getGain(){ return  gain; }

Eigen::Vector3d VirtualForceBrf::getForcePoint (geometry_msgs::Point32 & c_current, Eigen::Vector3d robot_velocity ) {

    Eigen::Vector3d obsVector;
    obsVector(0) = c_current.x ;
    obsVector(1)=  c_current.y ;
    obsVector(2) = c_current.z ;

//    obsVector(0) =  c_current.x * cos(yaw) - c_current.y * sin(yaw)  + CurrentRobotPose(0)  ;
//    obsVector(1)=  c_current.x * sin(yaw) + c_current.y * cos(yaw) + CurrentRobotPose(1)  ;
//    obsVector(2) = c_current.z + CurrentRobotPose(2) ;

    double obsMag = sqrt(pow(c_current.x, 2) + pow(c_current.y , 2) + pow(c_current.z , 2)) ;

    Eigen::Vector3d obsVecNorm = obsVector / obsMag ;


double a_max = 1.0 ;

    // ******************* force in x ******************************* //
    double v_ix = robot_velocity(0) ;
    float dstopx = (v_ix * v_ix) / (2*a_max ) ;
    float dresx ;
    if ( v_ix <= 0 ) dresx = obsMag + dstopx ;

    else dresx = obsMag - dstopx ;


    double f_x ;
    if ( dresx <= 0 || (1+v_ix)/dresx >= 1/gain)
        f_x =  1.0;
    else if ( (1+v_ix)/dresx <= 0)
        f_x  =  0.0 ;
    else
    {
        //std::cout << "risk vector" << gain *(1+v_i)/ dresx  << std::endl ;
        f_x =  (getGain()*((1+v_ix)/ dresx));
    }
// ************************* force in y ****************************** //
//   double v_y = robot_velocity(0) ;
//   float dstop = (v_y * v_y) / (2*a_max ) ;
//   float dres ;
//    if ( v_y <= 0 ) dres = obsVector(1) + dstop ;
//    else dres = obsVector(1) - dstop ;


//    double f_y ;

//    if ( dres <= 0 || (1+v_y)/dres >= 1/gain)
//        f_y =   1.0;
//    else if ( (1+v_y)/dres <= 0)
//        f_y  = 0.0 ;
//    else
//    {
//        //std::cout << "risk vector" << gain *(1+v_y)/ dres  << std::endl ;
//        f_y =  (getGain()*(1+v_y)/ dres);
//    }


// double f_z = 0.0 ;
//    Eigen::Vector3d  forceMag;
//    forceMag(0) = f_x ;
//    forceMag(1)=  f_y; //c_current.y ;
//    forceMag(2) = f_z; //c_current.z;


    //forceMag << f_x , f_y , 0.0 ;
   // Eigen::Vector3d  f= -obsVecNorm *forceMag.norm() ;
    Eigen::Vector3d  f= -obsVecNorm * f_x;

//    std::cout << "f_x  " << f_x << "  F_y " << f_y << std::endl ;
 //   std::cout << "f_x  " << f(0) << "  F_y " << f(1) << std::endl ;

    return f ;

}





