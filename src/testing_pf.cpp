#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <phantom_omni/OmniFeedback.h>
#include <dynamic_reconfigure/server.h>
#include <haptic_teleoperation/TwistArray.h>
#include <haptic_teleoperation/ContourData.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <haptic_teleoperation/potential_fieldConfig.h>
#include <phantom_omni/PhantomButtonEvent.h>
#include <gazebo_msgs/ModelState.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

//const double M_PI=3.14159265359 ;
const double deg_to_rad = M_PI / 180.0 ;
class test_pf
{
public:

    test_pf(ros::NodeHandle & n_) : n(n_)
    {

//        vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        model_state_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);

    };
    void init_model_state(std::string model_name, geometry_msgs::Pose & pose)
    {
        gazebo_msgs::ModelState robot_msg ;
        robot_msg.model_name = model_name;
        robot_msg.pose = pose;

        model_state_pub.publish(robot_msg) ;
    }

private:
    // ROS
    ros::NodeHandle n;


    ros::Publisher vel_pub ;
    ros::Publisher model_state_pub ;

    // Helper variables
    bool flag;





    //    void publish_velocity()
    //    {
    //        std::cout << "publish velocities: " << std::endl ;
    //        geometry_msgs::Twist msg;
    //        std::cout << "flag " << flag << std::endl ;


    //        if(flag){
    //            std::cout << "forward velocities: " << std::endl ;

    //            msg.linear.x =  0 ;
    //            msg.linear.y =  0.5 ;
    //            msg.linear.z =  0.0;
    //            msg.angular.z = 0 ;
    //            msg.angular.y = 0 ;
    //            msg.angular.z = 0 ;
    //        }
    //        else {
    //            std::cout << "backward velocities: " << std::endl ;
    //            msg.linear.x =  0.0 ;
    //            msg.linear.y =  -0.5 ;
    //            msg.linear.z =  0 ;
    //            msg.angular.z = 0 ;
    //            msg.angular.y = 0 ;
    //            msg.angular.z = 0 ;
    //        }

    //        vel_pub.publish(msg);



    //    }
} ;

int main(int argc, char **argv)
{


    std:: cout << " MAIN " << std::endl ;
    ros::init(argc, argv, "test_pf");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    ros::Rate loop_rate(5);
    test_pf potential_field(n);
    std::string robot_name = "quadrotor" ;
    std::string obj = "grey_wall" ;
    geometry_msgs::Pose robot_pose;
    geometry_msgs::Pose wall_pose;

    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(deg_to_rad*-90.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q(m) ;

    wall_pose.position.x=0.0 ;
    wall_pose.position.y=0.0 ;
    wall_pose.position.z=0.0 ;
    wall_pose.orientation.x=q.x() ;
    wall_pose.orientation.y=q.y() ;
    wall_pose.orientation.z=q.z() ;
    wall_pose.orientation.w=q.w() ;


    robot_pose.position.x=-5.0 ;
    robot_pose.position.y=0.0 ;
    robot_pose.position.z=2.0 ;
    robot_pose.orientation.x=0.0 ;
    robot_pose.orientation.y=0.0 ;
    robot_pose.orientation.z=0.0 ;
    robot_pose.orientation.w=1.0 ;



    bool first_time = true ;
    sleep(1);
    while(ros::ok())
    {
        if(first_time)
        {
            potential_field.init_model_state(robot_name, robot_pose );
            potential_field.init_model_state(obj,wall_pose );
            first_time=false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

