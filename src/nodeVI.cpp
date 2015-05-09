#include "haptic_teleoperation/VirtualImpedanceForce.h"
sensor_msgs::PointCloud obstacles_positions;
bool testFlag = true ;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vi");

    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    double freq;
    n_priv.param<double>("frequency", freq, 50.0);
    ros::Rate loop_rate(freq);


    // KP K D param //
    double kp_x;
    double kp_y;
    double kp_z;
    double kd_x;
    double kd_y;
    double kd_z;

    // Control gains
    n_priv.param<double>("Kp_x", kp_x, 1.0);
    n_priv.param<double>("Kp_y", kp_y, 1.0);
    n_priv.param<double>("Kp_z", kp_z, 1.0);
    n_priv.param<double>("Kp_x", kd_x, 20.0);
    n_priv.param<double>("Kp_y", kd_y, 20.0);
    n_priv.param<double>("Kp_z", kd_z, 20.0);

    Eigen::Vector3d kp(kp_x,kp_y,kp_z);
    Eigen::Vector3d kd(kd_x,kd_y,kd_z);


    std::cout << "Creatred Object " << std::endl ;
    VirtualImpedanceForce obj(n ,kp, kd);


    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


