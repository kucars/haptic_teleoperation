
#include "haptic_teleoperation/VirtualForcePrf.h"

VirtualForcePrf::VirtualForcePrf(ros::NodeHandle & n_):ForceField(n_)
{

std::cout << "child constructor" << std::endl ; 

}

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
Eigen::Vector3d VirtualForcePrf::getForcePoint (geometry_msgs::Point32 & c_current , Eigen::Vector3d  robot_vel) {

    //robot_vel[0] = 1 ;
    //robot_vel[1] = 0 ;
    //robot_vel[2] = 0 ;
    double rpz = getCriticalAreaRadius() ;
    double tahead = getTimeAhead();
    double amax = getMaxAcceleration() ;
    double dmin = getMinDistance() ;

    double dstop = (robot_vel(0) * robot_vel(0) + (robot_vel(1) * robot_vel(1)) + (robot_vel(2) * robot_vel(2)) ) / (2* amax) ;
    double dahead = sqrt((robot_vel(0) * robot_vel(0) + (robot_vel(1) * robot_vel(1)) + (robot_vel(2) * robot_vel(2)) ) ) * tahead ;
    Eigen::Vector3d obsVector;
    obsVector(0) = c_current.x ;
    obsVector(1)=  c_current.y ;
    obsVector(2) = c_current.z;
    double obsMag = sqrt(pow(c_current.x, 2) + pow(c_current.y , 2) + pow(c_current.z , 2)) ;
    Eigen::Vector3d obsVecNorm = obsVector / obsMag ;
    // norm of y
    double yi;// = abs(c_current.y) ;
    double signy;//  = signed(c_current.y) ;
        if (c_current.y>= 0 )
        {
            yi = c_current.y ;
            signy = 1 ;

        }
        else
       {
            yi = -c_current.y ;
            signy = -1 ;
        }
    //        // critical area
    if(        (c_current.x <=0 && sqrt(pow(c_current.x,2) + pow(c_current.y,2)) <= rpz)
            || ( (c_current.x > 0) &&( c_current.x <=dstop) && (yi<=rpz) )
            || ( (c_current.x > dstop) && (sqrt(pow((c_current.x - dstop),2) + pow(c_current.y,2)) <= rpz) )
      )
    {

        Eigen::Vector3d f= -obsVecNorm *1 ;
        return f ;
    }
    else if ( (c_current.x <= 0) && (sqrt(pow(c_current.x,2) + pow(c_current.y,2)) >rpz) && (sqrt(pow(c_current.x,2) + pow(c_current.y,2)) < (rpz + dmin)))
    {
        double d = sqrt(pow(c_current.x,2) + pow(c_current.y,2)) - rpz  ;
        double d0 = dmin ;
        Eigen::Vector3d f= -obsVecNorm *(cos(((d/d0) * (PI/2)) + (PI/2)) + 1 );
        //   Eigen::Vector3d f= -obsVecNorm*((0.5 *cos((d/d0) * PI)) + 0.5) ;

        return f ;
    }
    else if ( c_current.x >=0 && c_current.x <=dstop && (yi>rpz)&&  yi <= (rpz+dmin))
    {
        double d =yi- rpz ;
        double d0 = dmin ;
        Eigen::Vector3d f= -obsVecNorm *(cos(((d/d0) * (PI/2)) + (PI/2)) + 1 ) ;
        return f ;
    }

    else if ( c_current.x>=dstop && sqrt( pow((c_current.x - dstop),2) + pow(c_current.y,2) ) >= rpz && c_current.x <= (dstop+dahead) &&  yi <= (rpz+dmin) && yi >= (((rpz+dmin) / dahead )* (c_current.x - dstop)))

//    else if ( c_current.x>=dstop && sqrt( pow((c_current.x - dstop),2) + pow(c_current.y,2) ) >= rpz && c_current.x <= (dstop+dahead) &&  yi <= (rpz+dmin) && yi >= (((rpz+dmin) / dahead )* (c_current.x - dstop)))
    {

        double d = sqrt(pow(c_current.x-dstop,2) + pow(c_current.y,2)) - rpz ;

        //double theta = atan( yi / (c_current.x - dstop)) ;
        //double phi = 90 - theta;
       // double xx= ((rpz+dmin)/sin(theta)) * sin(phi);
       // double d0 = sqrt( pow(xx,2) + pow(rpz+dmin ,2)) - rpz ;
        double youtline = (rpz + dmin )* signy ;
        double xoutline = ((c_current.x - dstop) / c_current.y ) * youtline ; (youtline*(c_current.x - dstop)/c_current.y ) + dstop ;

//        double youtline = (rpz + dmin )* signy ;
//        double xoutline ;
//        if (c_current.y == 0 )
//            xoutline = dahead + dstop ;
//        else
//            xoutline = (youtline*(c_current.x - dstop)/c_current.y ) + dstop ;

//        if ( xoutline >= dahead)
//        {
//            double R = dmin + rpz ;
//            double theta = atan(yi / (c_current.x -dstop)) ;
//            double phi = asin((dahead * sin(theta)) / R) ;
//            double gama = phi + theta  ;
//            xoutline = R * cos(gama) + dahead + dstop ;
//            youtline = R * sin(gama) * signy ;


//        }
 //       double d = sqrt (pow((c_current.x - dstop),2) + pow(c_current.y,2)) - rpz ;
//        //double d0 = sqrt( (xoutline-dstop)*(xoutline-dstop) + youtline * youtline) - rpz ;
        double d0 = sqrt( pow(xoutline,2) + pow(youtline,2)) - rpz ;

        Eigen::Vector3d f= -obsVecNorm *(cos(((d/d0) * (PI/2)) + (PI/2)) + 1 ) ;
        return f ;
    }


    else if (((c_current.x >= (dstop + dahead)) &&  sqrt(pow((c_current.x - (dstop+dahead)),2) + pow(c_current.y,2)) <= (dmin+rpz) ) ||
             (
                 c_current.x >= dstop &&
                 c_current.x < dahead+dstop &&
                 sqrt(pow((c_current.x - dstop),2) + pow(c_current.y,2) ) >= rpz &&
                 yi < (((rpz+dmin) / dahead )* (c_current.x - dstop))
              )
             )
    {
        double R = dmin + rpz ;
        double theta = atan(yi/ (c_current.x -dstop)) ;
        double phi = asin((dahead * sin(theta)) / R) ;
        double gama = phi + theta  ;
        double xoutline = (R * cos(gama)) + dahead + dstop   ;
        double youtline = (R * sin(gama)) * signy  ;
        double d = sqrt(pow((c_current.x - dstop),2) + pow(c_current.y,2)) - rpz ;
        //  double d0 = sqrt( (xoutline-dstop)*(xoutline-dstop) + youtline * youtline) - rpz ;
        double d0 = sqrt(pow(xoutline-dstop,2) + pow(youtline,2)) - rpz ;

        Eigen::Vector3d f= -obsVecNorm *(cos(((d/d0) * (PI/2)) + (PI/2)) + 1 ) ;

        return f ;
    }

    else
        return Eigen::Vector3d(0,0,0);
}





