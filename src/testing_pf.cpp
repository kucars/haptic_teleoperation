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
<<<<<<< HEAD
//#include <haptic_teleoperation/TwistArray.h>
//#include <haptic_teleoperation/ContourData.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
//#include <haptic_teleoperation/potential_fieldConfig.h>
=======
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
>>>>>>> fbc08eb2950cff5a030a853ff06d3457b56719f5
#include <phantom_omni/PhantomButtonEvent.h>
#include <gazebo_msgs/ModelState.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

//const double M_PI=3.14159265359 ;

const double deg_to_rad = M_PI / 180.0 ;

class test_pf
{
public:
    bool flag_out;
    bool flag_first_Read;

    Eigen::Matrix<double,3,1> pose_slave;
    // Eigen::Matrix<double,3,1> pose_slave_2;

    test_pf(ros::NodeHandle & n_) : n(n_),flag_out(true),flag_first_Read(true)
    {
        vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        //       robot_pose_sub = n.subscribe("/ground_truth/state", 1, &test_pf::slaveOdometryCallback, this);
        //       model_state_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);
    }

    //    void init_model_state(std::string model_name, geometry_msgs::Pose & pose)
    //    {
    //        gazebo_msgs::ModelState robot_msg ;
    //        robot_msg.model_name = model_name;
    //        robot_msg.pose = pose;
    //        model_state_pub.publish(robot_msg) ;
    //    }

    void vel_init(double val)
    {
        geometry_msgs::Twist msg;

        //    std::cout << "A" << std::endl ;
        msg.linear.x =  0.0;
        msg.linear.y =  0.0 ;
        msg.linear.z =  val;
        msg.angular.x = 0 ;
        msg.angular.y = 0 ;
        msg.angular.z = 0 ;
        vel_pub.publish(msg);

    }

    //    void move(geometry_msgs::Pose & pose)
    //    {
    //        geometry_msgs::Twist msg;
    //        std::cout << "abs(pose_slave(1)-pose.position.y)" << fabs(pose_slave(0)-pose.position.x) << std::endl ;
    //        std::cout << "(pose_slave(1))" << pose_slave(0) << std::endl ;
    //        std::cout << "(pose.position.x)" << pose.position.x<< std::endl ;

    //        // flag_out = true;
    //        std::cout << "flag" << flag_out << std::endl ;
    //        if (!flag_first_Read )
    //        {
    //            if (fabs(pose_slave(0)-pose.position.x) > 2.0 && flag_out )
    //            {
    //                std::cout << "A" << std::endl ;

    //                msg.linear.x =  0.5;
    //                msg.linear.y =  0.0 ;
    //                msg.linear.z =  0.0;
    //                msg.angular.x = 0 ;
    //                msg.angular.y = 0 ;
    //                msg.angular.z = 0 ;
    //            }

    //            else
    //            {
    //                flag_out = false;
    //                if(fabs(pose_slave(0)-pose.position.x) > 0.5)
    //                    std::cout<<"something"<<std::endl;
    //                std::cout << "B" << std::endl ;
    //                msg.linear.x =  -0.5;
    //                msg.linear.y =  0.0 ;
    //                msg.linear.z =  0.0;
    //                msg.angular.x = 0 ;
    //                msg.angular.y = 0 ;
    //                msg.angular.z = 0 ;
    //            }

    //            vel_pub.publish(msg);
    //        }
    //    }

private:
    // ROS
    ros::NodeHandle n;
    ros::Publisher vel_pub ;
    ros::Publisher model_state_pub ;
    ros::Subscriber robot_pose_sub;
    // Helper variables

    //  void slaveOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

} ;
//void test_pf::slaveOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
//{
//    if (flag_first_Read)
//    {
//        flag_first_Read =false;
//    }
//    pose_slave << msg->pose.pose.position.x,
//            msg->pose.pose.position.y,
//            msg->pose.pose.position.z;
//}


int main(int argc, char **argv)
{
    std:: cout << " MAIN " << std::endl ;
    ros::init(argc, argv, "test_pf");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    ros::Rate loop_rate(100);
    test_pf potential_field(n);
    // std::string robot_name = "quadrotor" ;
    //    std::string robot_name_2 = "mobile_base" ;
    //    std::string obj = "grey_wall" ;
    //    std::string obj_0 = "grey_wall_0" ;

    //    geometry_msgs::Pose robot_pose;
    //    geometry_msgs::Pose robot_pose_2;

    //    geometry_msgs::Pose wall_pose;
    //    geometry_msgs::Pose wall_pose_0;


    //    Eigen::Matrix3d m;
    //    m = Eigen::AngleAxisd(deg_to_rad*0, Eigen::Vector3d::UnitZ());
    //    Eigen::Quaterniond q(m) ;

    //    Eigen::Matrix3d m2;
    //    m2 = Eigen::AngleAxisd(deg_to_rad*90.0, Eigen::Vector3d::UnitZ());
    //    Eigen::Quaterniond q2(m2) ;



    //    robot_pose.position.x=4.0 ;
    //    robot_pose.position.y=0.0 ;
    //    robot_pose.position.z=1.5 ;
    //    robot_pose.orientation.x=q.x() ;
    //    robot_pose.orientation.y=q.y() ;
    //    robot_pose.orientation.z=q.z();
    //    robot_pose.orientation.w=q.w();


    //    robot_pose_2.position.x=-10.0 ;
    //    robot_pose_2.position.y=-10.0 ;
    //    robot_pose_2.position.z=0.0 ;
    //    robot_pose_2.orientation.x=q.x() ;
    //    robot_pose_2.orientation.y=q.y() ;
    //    robot_pose_2.orientation.z=q.z();
    //    robot_pose_2.orientation.w=q.w();

    //    wall_pose.position.x=0.0 ;
    //    wall_pose.position.y=0.0 ;
    //    wall_pose.position.z=0.0 ;
    //    wall_pose.orientation.x=q2.x() ;
    //    wall_pose.orientation.y=q2.y() ;
    //    wall_pose.orientation.z=q2.z() ;
    //    wall_pose.orientation.w=q2.w() ;

    //    wall_pose_0.position.x=-10.0 ;
    //    wall_pose_0.position.y=-2.5 ;
    //    wall_pose_0.position.z=0.0 ;
    //    wall_pose_0.orientation.x=q2.x() ;
    //    wall_pose_0.orientation.y=q2.y() ;
    //    wall_pose_0.orientation.z=q2.z() ;
    //    wall_pose_0.orientation.w=q2.w() ;
    // sleep(2);
    // potential_field.init_model_state(robot_name, robot_pose );
    // potential_field.init_model_state(robot_name_2, robot_pose_2 );
    // sleep(2);
    // potential_field.init_model_state(obj,wall_pose );
    //  sleep(2);
    // potential_field.init_model_state(obj_0,wall_pose_0 );


    for(int i=1;i<800000;i++)
    {
        potential_field.vel_init(0.2);
    }
    sleep(5);

    for (int i =1 ; i < 2000000 ; i++ )
    {
        potential_field.vel_init(-0.2);
    }
    sleep(2);

    potential_field.vel_init(0.0);

    // sleep(2);


    std::cout << "cancle now" << std::endl ;
    // exit(1) ;
    //   sleep(2);


    while(ros::ok())
    {
<<<<<<< HEAD
        //potential_field.move(wall_pose);
=======
        // potential_field.move(wall_pose);
>>>>>>> 880f690c69a4b559264f4910d84b4859e17aa6d3
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

