
#include "haptic_teleoperation/VirtualForcePrf.h"

VirtualForcePrf::VirtualForcePrf(ros::NodeHandle & n_):ForceField(n_)
{std::cout << "child constructor" << std::endl ; }

void VirtualForcePrf::setParameters(double dmin, double amax, double rpz, double tahead)
{
    setMinDistance(dmin);
    setMaxAcceleration(amax);
    setCriticalAreaRadius(rpz);
    setTimeAhead(tahead);

}
void VirtualForcePrf::setMinDistance(double dmin)
{
    minDistance= dmin ;
}

void  VirtualForcePrf::setMaxAcceleration( double amax)
{
    maxAcceleration = amax ;
}

void  VirtualForcePrf::setCriticalAreaRadius(double rpz)
{
    criticalAreaRadius = rpz   ;
}

void  VirtualForcePrf::setTimeAhead(double tahead)
{
    timeAhead  = tahead;
}


double  VirtualForcePrf::getMinDistance(){return minDistance;}
double  VirtualForcePrf::getMaxAcceleration(){return maxAcceleration;}
double  VirtualForcePrf::getCriticalAreaRadius() { return criticalAreaRadius; }
double  VirtualForcePrf::getTimeAhead(){ return timeAhead; }

Eigen::Vector3d VirtualForcePrf::getForcePoint (Eigen::Vector3d & c_current) {

    robot_vel[0] = 1 ;
    robot_vel[1] = 0 ;
    robot_vel[2] = 0 ;
    double rpz = getCriticalAreaRadius() ;
    double tahead = getTimeAhead();
    double amax = getMaxAcceleration() ;
    double dmin = getMinDistance() ;

    double dstop = ((robot_vel[0] * robot_vel[0]) + (robot_vel[1] * robot_vel[1]) + (robot_vel[2] * robot_vel[2]) ) / (2* amax) ;
    double dahead = sqrt(((robot_vel[1] * robot_vel[1]) + (robot_vel[2] * robot_vel[2]) + (robot_vel[2] * robot_vel[2]) )) * tahead ;

    // norm of y
    double yi ;
    double signy  ;
    if (c_current(1)>= 0 )
    {
        yi = c_current(1) ;
        signy = 1 ;

    }
    else
    {
        yi = -c_current(1) ;
        signy = -1 ;
    }
    //        // critical area
    if( (c_current(0) <=0 && sqrt(c_current(0)* c_current(0) + c_current(1)*c_current(1)) <= rpz)
            || ( (c_current(0) >= 0) &&( c_current(0) <=dstop) && (yi<=rpz) )
            || (c_current(0) > dstop && sqrt((c_current(0) - dstop)*(c_current(0) - dstop) + c_current(1)*c_current(1)) <= rpz))
    {

        Eigen::Vector3d f= -c_current.normalized() *1 ;
           return f ;
    }
        else if ( c_current(0) <= 0 && sqrt(c_current(0)*c_current(0) + c_current(1) * c_current(1)) >rpz && sqrt(c_current(0)*c_current(0) + c_current(1) * c_current(1)) < (rpz + dmin))
    {
        double d = sqrt(c_current(0)*c_current(0) + c_current(1) * c_current(1)) - rpz  ;
        double d0 = dmin ;
        Eigen::Vector3d f= -c_current.normalized() *(cos(((d/d0) * PI/2) + (PI/2)) + 1 );
     //   Eigen::Vector3d f= -c_current.normalized()*((0.5 *cos((d/d0) * PI)) + 0.5) ;

        return f ;
    }
    else if ( c_current(0) >=0 && c_current(0) <=dstop &&  yi <= (rpz+dmin))
    {
        double d =yi- rpz ;
        double d0 = dmin ;
  //      Eigen::Vector3d f= -c_current.normalized()*((0.5 *cos((d/d0) * PI)) + 0.5) ;

        Eigen::Vector3d f= -c_current.normalized() *(cos(((d/d0) * PI/2) + (PI/2)) + 1 );
        return f ;
    }


    else if ( c_current(0)>=dstop && sqrt(((c_current(0) - dstop)*(c_current(0) - dstop)) + (c_current(1) * c_current(1)) ) > rpz && c_current(0) <= (dstop+dahead ) && yi < (rpz+dmin))
    {
        double youtline = (rpz + dmin )* signy ;
        double xoutline ;
        if (c_current(1) == 0 )
            xoutline = dahead + dstop ;
        else
            xoutline = (youtline*(c_current(0) - dstop)/c_current(1) ) + dstop ;

        if ( xoutline >= dahead || ( (c_current(0) >= (dstop + dahead)) &&  sqrt  (   (    (c_current(0) - (dstop+dahead)) * (c_current(0) - (dstop+dahead))) + (c_current(1)* c_current(1))) <= (dmin+rpz)  ))
        {
            double R = dmin + rpz ;
            double theta = atan(c_current(1) / (c_current(0) -dstop)) ;
            double phi = asin(dahead * sin(theta) / R) ;
            double gama = phi + theta  ;
            double xoutline = R * cos(gama) + dahead + dstop ;
            double youtline = R * sin(gama) * signy ;
            double d = sqrt (((c_current(0) - dstop) * (c_current(0) - dstop)) + (c_current(1) * c_current(1))) - rpz ;
            double d0 = sqrt( (xoutline-dstop)*(xoutline-dstop) + youtline * youtline) - rpz ;
    //        Eigen::Vector3d f= -c_current.normalized()*((0.5 *cos((d/d0) * PI)) + 0.5) ;

            Eigen::Vector3d f= -c_current.normalized() *(cos(((d/d0) * PI/2) + (PI/2)) + 1 );
            return f ;
        }
        else {
            double d = sqrt(((c_current(0) - dstop ) * (c_current(0) - dstop )) + (c_current(1) * c_current(1) )) ;
            double d0 = sqrt( xoutline*xoutline + youtline * youtline) - rpz ;
            Eigen::Vector3d f= -c_current.normalized() *(cos(((d/d0) * PI/2) + (PI/2)) + 1 );
         //   Eigen::Vector3d f= -c_current.normalized()*((0.5 *cos((d/d0) * PI)) + 0.5) ;

            return f ;
        }
    }
    else if ( (c_current(0) >= (dstop + dahead)) &&  sqrt  (   (    (c_current(0) - (dstop+dahead)) * (c_current(0) - (dstop+dahead))) + (c_current(1)* c_current(1))) <= (dmin+rpz)  )
    {
        double R = dmin + rpz ;
        double theta = atan(c_current(1) / (c_current(0) -dstop)) ;
        double phi = asin(dahead * sin(theta) / R) ;
        double gama = phi + theta  ;
        double xoutline = R * cos(gama) + dahead + dstop ;
        double youtline = R * sin(gama) * signy  ;
        double d = sqrt(((c_current(0) - dstop) * (c_current(0) - dstop)) + (c_current(1) * c_current(1))) - rpz ;
        double d0 = sqrt( (xoutline-dstop)*(xoutline-dstop) + youtline * youtline) - rpz ;
        Eigen::Vector3d f= -c_current.normalized() *(cos(((d/d0) * PI/2) + (PI/2)) + 1 );
      //  Eigen::Vector3d f= -c_current.normalized()*((0.5 *cos((d/d0) * PI)) + 0.5) ;

        return f ;
    }

    else
        return Eigen::Vector3d(0,0,0);
}





