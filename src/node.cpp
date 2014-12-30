#include "haptic_teleoperation/VirtualForcePrf.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prf");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    double freq;
    n_priv.param<double>("frequency", freq, 50.0);
    ros::Rate loop_rate(freq);

//    double dmin= 1.2 ;
//    double amax= 1.0 ;
//    double rpz = 0.4 ;
//    double  tahead = 2 ;


    double dmin= 0.6 ;
    double amax= 1.0 ;
    double rpz = 0.2 ;
    double  tahead = 2 ;




    VirtualForcePrf prf_obj(n);
    prf_obj.setParameters(dmin,amax,rpz,tahead) ;
    prf_obj.runTest();

    while(ros::ok())
    {
        std::cout << "loop" << std::endl;
        ros::spin();
        loop_rate.sleep();
    }

    return 0;
}
