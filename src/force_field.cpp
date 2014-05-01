#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <cmath>
#include <phantom_omni/OmniFeedback.h>
#include <dynamic_reconfigure/server.h>
#include <navigation/ForceFieldConfig.h>

const double PI=3.14159265359;

class ForceField
{
public:
    dynamic_reconfigure::Server<navigation::ForceFieldConfig> param_server;
    dynamic_reconfigure::Server<navigation::ForceFieldConfig>::CallbackType param_callback_type;

    ForceField(ros::NodeHandle & n_, double & freq_, double & ro_,   Eigen::Vector3d kp_, Eigen::Vector3d kd_, double & laser_max_distance_, double & robot_mass_, double & robot_radius_, std::string & pose_topic_name_, std::string & sonar_topic_name_) : n(n_), freq(freq_), ro(ro_), kp(kp_), kd(kd_), laser_max_distance(laser_max_distance_), robot_mass(robot_mass_), robot_radius(robot_radius_), pose_topic_name(pose_topic_name_), sonar_topic_name(sonar_topic_name_)
    {

        kp_mat << kp.x(), 0, 0,
                0, kp.y(), 0,
                0, 0, kp.z();
        kd_mat << kd.x(), 0, 0,
                0, kd.y(), 0,
                0, 0, kd.z();

        param_callback_type = boost::bind(&ForceField::paramsCallback, this, _1, _2);
        param_server.setCallback(param_callback_type);

        visualization_markers_pub = n.advertise<visualization_msgs::MarkerArray>( "force_field_markers", 1);
        //velocity_cmd_pub = n.advertise<geometry_msgs::Twist>( "/cmd_vel", 1);
        repulsive_force_out = n.advertise<geometry_msgs::Twist>( "/potential_field/repulsive_force", 1);

        force_out = n.advertise<phantom_omni::OmniFeedback>( "/omni1_force_feedback", 1);

        init_flag=false;

        obstacle_readings_sub = n.subscribe("cloud", 1, &ForceField::sonarCallback, this);
    };

    void computeForceField()
    {
        resulting_force=Eigen::Vector3d(0.0,0.0,0.0);

        // Compute current robot Velocity based on odometry readings
        std::vector<Eigen::Vector3d> force_field;

        // for each obstacle compute velocity with respect to that object
        unsigned int aux_it;
        if(obstacles_positions_current.size()<=obstacles_positions_current.size())
            aux_it=obstacles_positions_current.size();
        else
            aux_it=obstacles_positions_previous.size();

        for(int i=0; i<aux_it; ++i)
        {
            force_field.push_back(getForcePoint(obstacles_positions_current[i], obstacles_positions_previous[i], ro));
            force_field[i] = force_field[i] / aux_it ;
            resulting_force+=force_field[i];
        }
        //std::cout << "resulting force: " << resulting_force.transpose() << std::endl;
        // Publish visual markers to see in rviz
        visualization_msgs::MarkerArray marker_array=rviz_arrows(force_field, obstacles_positions_current, std::string("force_field"));
        visualization_msgs::Marker marker=rviz_arrow(resulting_force, Eigen::Vector3d(0,0,0), 0,   std::string("resulting_force"));
        marker_array.markers.push_back(marker);
        visualization_markers_pub.publish(marker_array);
    }

    Eigen::Vector3d getForcePoint(Eigen::Vector3d & c_current, Eigen::Vector3d & c_previous, const double & ro)
    {
        if(c_current.norm()<ro)
        {
            Eigen::Vector3d f=kp_mat*(ro-c_current.norm())*c_current.normalized()
                    -kd_mat*(c_current.norm()-c_previous.norm())*c_current.normalized();

            return f;
        }
        else
        {
            return Eigen::Vector3d(0,0,0);
        }
    }

private:
    // ROS
    ros::NodeHandle n;

    ros::Subscriber obstacle_readings_sub;
    //ros::Publisher velocity_cmd_pub;
    ros::Publisher visualization_markers_pub;
    ros::Publisher force_out;
    ros::Publisher repulsive_force_out;

    std::string pose_topic_name;
    std::string sonar_topic_name;

    std::string velocity_cmd_topic_name;

    // Parameters
    double a_max;
    double ro;
    double freq;
    double robot_mass;
    double robot_radius;
    double laser_max_distance;

    Eigen::Vector3d kp, kd;
    Eigen::Matrix3d kp_mat, kd_mat;

    // Helper variables
    bool init_flag;

    std::vector<Eigen::Vector3d> obstacles_positions_current;
    std::vector<Eigen::Vector3d> obstacles_positions_previous;

    ros::Time previous_time;
    int count;

    Eigen::Vector3d resulting_force;

    void paramsCallback(navigation::ForceFieldConfig &config, uint32_t level)
    {
        ROS_DEBUG_STREAM("Force field reconfigure Request ->" << " kp_x:" << config.kp_x
                         << " kp_y:" << config.kp_y
                         << " kp_z:" << config.kp_z
                         << " kd_x:" << config.kd_x
                         << " kd_y:" << config.kd_y
                         << " kd_z:" << config.kd_z
                         << " ro:"   << config.ro);

        kp << config.kp_x,
                config.kp_y,
                config.kp_z;

        kd << config.kd_x,
                config.kd_y,
                config.kd_z;

        kp_mat << kp.x(), 0, 0,
                0, kp.y(), 0,
                0, 0, kp.z();
        kd_mat << kd.x(), 0, 0,
                0, kd.y(), 0,
                0, 0, kd.z();

        ro = config.ro;

        laser_max_distance = config.laser_max_distance;
        //slave_to_master_scale=Eigen::Matrix<double,3,1> (fabs(config.master_workspace_size.x/config.slave_workspace_size.x), fabs(config.master_workspace_size.y/config.slave_workspace_size.y), fabs(config.master_workspace_size.z/config.slave_workspace_size.z));
    }

    visualization_msgs::Marker rviz_arrow(const Eigen::Vector3d & arrow, const Eigen::Vector3d & arrow_origin, int id, std::string name_space )
    {
        Eigen::Quaternion<double> rotation;
        if(arrow.norm()<0.0001)
        {
            rotation=Eigen::Quaternion<double>(1,0,0,0);
        }
        else
        {
            double rotation_angle=acos(arrow.normalized().dot(Eigen::Vector3d::UnitX()));
            Eigen::Vector3d rotation_axis=arrow.normalized().cross(Eigen::Vector3d::UnitX()).normalized();
            rotation=Eigen::AngleAxisd(-rotation_angle+PI,rotation_axis);
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = "Pioneer3AT/base_link";
        marker.header.stamp = ros::Time();
        marker.id = id;
        if(id==0)
        {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.ns = name_space;
        }
        else
        {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.ns = name_space;
        }
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = arrow_origin.x();
        marker.pose.position.y = arrow_origin.y();
        marker.pose.position.z = arrow_origin.z();
        marker.pose.orientation.x = rotation.x();
        marker.pose.orientation.y = rotation.y();
        marker.pose.orientation.z = rotation.z();
        marker.pose.orientation.w = rotation.w();
        //std::cout <<"position:" <<marker.pose.position << std::endl;
        //std::cout <<"orientation:" <<marker.pose.orientation << std::endl;
        //marker.pose.orientation.x = 0;
        //marker.pose.orientation.y = 0;
        //marker.pose.orientation.z = 0;
        //marker.pose.orientation.w = 1;
        if(arrow.norm()<0.0001)
        {
            marker.scale.x = 0.001;
            marker.scale.y = 0.001;
            marker.scale.z = 0.001;
        }
        else
        {
            marker.scale.x = arrow.norm();
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
        }
        marker.color.a = 1.0;


        return marker;
    }

    visualization_msgs::MarkerArray rviz_arrows(const std::vector<Eigen::Vector3d> & arrows, const std::vector<Eigen::Vector3d> arrows_origins, std::string name_space)
    {
        visualization_msgs::MarkerArray marker_array;
        for(int i=0; i< arrows.size();++i)
        {
            marker_array.markers.push_back(rviz_arrow(arrows[i], arrows_origins[i], (i+1), name_space));
        }
        return marker_array;
    }



    void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
    {
        obstacles_positions_current.clear();
        for(int i=0; i< msg->points.size(); ++i)
        {
            Eigen::Vector3d obstacle(msg->points[i].x,msg->points[i].y,0.0);
            if(obstacle.norm()<laser_max_distance-0.01)
                //if((obstacle.norm()>robot_radius)&&(obstacle.norm()<laser_max_distance-0.01)) // check if measurement is between the laser range and the robot
            {
                //ROS_INFO_STREAM("INSIDE THE LIMITS:"<<obstacle.norm());
                obstacles_positions_current.push_back(obstacle);
            }
        }

        if(!init_flag)
        {
            init_flag=true;
            obstacles_positions_previous=obstacles_positions_current;
            return;
        }
        //ROS_INFO_STREAM("obstacles:" << obstacles_positions.size());
        //ROS_INFO("I heard sensor data : [%f, %f , %f]", msg->points[0].x , msg->points[0].y , msg->points[0].z  );

        computeForceField();
        feedbackMaster();
        //feedbackSlave();
        obstacles_positions_previous=obstacles_positions_current;
    }

    // AUTONOMOUS CASE
    /*void feedbackSlave()
    {
        // Compute linear velocity (x velocity)
        ros::Time current_time=ros::Time::now();
        double period=current_time.toSec()-previous_time.toSec();
        previous_time=current_time;
        double threshold=0.2;
        if(period>threshold)
            return;

        double linear_speed=resulting_force.x()*period/(robot_mass); // LINEAR SPEED IS GIVEN BY THE PROJECTION OF THE FORCE IN X (normal component)

        double angular_speed=(resulting_force.y()/(robot_mass*robot_radius))*period; // ROTATIONAL SPEED IS GIVEN BY THE PROJECTION OF THE FORCE IN Y (perpendicular component)
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x=linear_speed;
        twist_msg.angular.z=angular_speed;
        velocity_cmd_pub.publish(twist_msg);
    }*/

    void feedbackMaster()
    {
        // WEIRD MAPPING!!!
        phantom_omni::OmniFeedback force_feedback;
        force_feedback.force.x=resulting_force.y();
        force_feedback.force.y=resulting_force.z();
        force_feedback.force.z=resulting_force.x();
        force_out.publish(force_feedback);

        geometry_msgs::Twist twist_msg_resulting_force;
        twist_msg_resulting_force.linear.x= resulting_force.y();
        twist_msg_resulting_force.linear.y=resulting_force.x();
        twist_msg_resulting_force.linear.z=resulting_force.z();

        repulsive_force_out.publish(twist_msg_resulting_force);

    }
};

int main(int argc, char **argv)
{
    /**
       * The ros::init() function needs to see argc and argv so that it can perform
       * any ROS arguments and name remapping that were provided at the command line. For programmatic
       * remappings you can use a different version of init() which takes remappings
       * directly, but for most command-line programs, passing argc and argv is the easiest
       * way to do it.  The third argument to init() is the name of the node.
       *
       * You must call one of the versions of ros::init() before using any other
       * part of the ROS system.
    */
    ros::init(argc, argv, "potential_field");
    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle n;

    ros::NodeHandle n_priv("~");

    // parameters
    double a_max;
    double ro;
    double freq;
    double robot_mass;
    double robot_radius;


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
    n_priv.param<double>("Kp_x", kd_x, 1.0);
    n_priv.param<double>("Kp_y", kd_y, 1.0);
    n_priv.param<double>("Kp_z", kd_z, 1.0);

    Eigen::Vector3d kp(kp_x,kp_y,kp_z);
    Eigen::Vector3d kd(kd_x,kd_y,kd_z);

    double laser_max_distance;

    //initialize operational parameters
    n_priv.param<double>("laser_max_distance", laser_max_distance, 0.2);
    n_priv.param<double>("ro", ro, 3.0);
    n_priv.param<double>("frequency", freq, 10.0);
    n_priv.param<double>("acc_max", a_max, 1.0);
    n_priv.param<double>("robot_mass", robot_mass, 1.0);
    n_priv.param<double>("robot_radius", robot_radius, 0.2);


    ros::Rate loop_rate(freq);

    std::string pose_topic_name = "/RosAria/pose" ;
    std::string sonar_topic_name = "/RosAria/sonar";
    ForceField potential_field(n, freq, ro, kp, kd, laser_max_distance, robot_mass, robot_radius, pose_topic_name, sonar_topic_name);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
