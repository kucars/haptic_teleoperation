#include "haptic_teleoperation/VirtualForceBrf.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "brf");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    double freq;
    n_priv.param<double>("frequency", freq, 50.0);
    ros::Rate loop_rate(freq);

    double gain[8] = {0.2, 0.4,0.6,0.8,1.0, 1.2 , 1.4 , 1.6 } ;





    VirtualForceBrf prf_obj(n);


    while(ros::ok())
    {
        std::cout << "loop" << std::endl;
        ros::spin();
        loop_rate.sleep();
    }

    return 0;
}
