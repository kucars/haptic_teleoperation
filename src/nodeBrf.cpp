#include "haptic_teleoperation/VirtualForceBrf.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "brf");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    double freq;
    n_priv.param<double>("frequency", freq, 50.0);
    ros::Rate loop_rate(freq);

    //double gain = 0.5 ;
    double gain = 1.0 ;

    VirtualForceBrf brf_obj(n);
    brf_obj.setGain(gain) ;

// ************* testing in open CV **************** //

    //double g    VirtualForceBrf brf_obj(n);
    //VirtualForceBrf brf_obj(n);

//    Eigen::Vector3d Robo_vel ;

//    for (int j=0 ; j<5 ; j++)
//    {
//        //std::cout << "J" << j << std::endl ;
//        Robo_vel(0) = (double) j/2 ;
//        Robo_vel(1) = 0 ;
//        Robo_vel(2) = 0 ;
//        brf_obj.setRobotVelocity(Robo_vel);

//        for ( int i = 0 ; i < 5; i++ )
//        {
//           string name = brf_obj.testNameBRF(gain[i], Robo_vel(0)) ;
//           std::cout << "name of test " << name << std::cout ;
//            brf_obj.setGain(gain[i]) ;
//            brf_obj.runTestBrf(name) ;
//        }
//    }

// ****************** END **************************** //

    while(ros::ok())
    {
        std::cout << "loop" << std::endl;
        ros::spin();
        loop_rate.sleep();
    }
    return 0;
}
