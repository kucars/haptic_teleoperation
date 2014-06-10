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
#include <navigation/TwistArray.h>
#include <navigation/ContourData.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <navigation/potential_fieldConfig.h>
#include <phantom_omni/PhantomButtonEvent.h>


using namespace std ; 
navigation::ContourData contour_data_msg;
const double PI=3.14159265359;
#define BILLION 1000000000
double lastTimeCalled ;//= ros::Time::now().toSec();
//bool linear_button_pressed = false ;

class ForceField
{
public:
    dynamic_reconfigure::Server<navigation::potential_fieldConfig> param_server;
    dynamic_reconfigure::Server<navigation::potential_fieldConfig>::CallbackType param_callback_type;
    std::vector<double> previous_potential_field;
    ros::Time previous_time;
    bool obstacles_new_readings;
    bool odometry_new_readings;
    double gain;
    std::vector<double> robot_position;

    ForceField(ros::NodeHandle & n_, double & freq_, double & ro_, double & gain_ ,  Eigen::Vector3d kp_, Eigen::Vector3d kd_, double & laser_min_distance_, double & laser_max_distance_, double & robot_mass_, double & robot_radius_, std::string & pose_topic_name_, std::string & sonar_topic_name_) : n(n_), freq(freq_), ro(ro_), gain(gain_),  kp(kp_), kd(kd_), laser_min_distance(laser_min_distance_), laser_max_distance(laser_max_distance_), robot_mass(robot_mass_), robot_radius(robot_radius_), pose_topic_name(pose_topic_name_), sonar_topic_name(sonar_topic_name_), odometry_new_readings(false), obstacles_new_readings(false)
    {
        std::cout << "new force field object" << std::endl;
        gain=1.0;
        a_max=1.0;

        kp_mat << kp.x(), 0, 0,
                0, kp.y(), 0,
                0, 0, kp.z();
        kd_mat << kd.x(), 0, 0,
                0, kd.y(), 0,
                0, 0, kd.z();

        std::cout << kp_mat << std::endl ;
        std::cout << std::endl ;
        std::cout << kd_mat << std::endl ;
        std::cout << std::endl ;
        param_callback_type = boost::bind(&ForceField::paramsCallback, this, _1, _2);
        param_server.setCallback(param_callback_type);
        visualization_markers_pub = n.advertise<visualization_msgs::MarkerArray>("/risk_vector_marker", 1);
        //force_out = n.advertise<phantom_omni::OmniFeedback>( "/omni1_force_feedback", 1);
        feedback_pub = n.advertise<geometry_msgs::PoseStamped>("/pf_force_feedback", 1);
        init_flag=false;
        obstacle_readings_sub = n.subscribe("/cloud",1, &ForceField::sonarCallback, this);
        // button_sub = n_.subscribe ("/omni1_button", 1, &ForceField::buttonCallback, this);

        std::cout << "end of the constructor" << std::endl;

        lastTimeCalled = ros::Time::now().toSec();

    };


    /* This function compute the potential field for each point and it sums all the points, then it takes the gradiant.
It is only going to be called when the robot sence the exiatance of the obstacle */

    void computePotentialField()
    {
        std::cout << "potential callback start 1" << std::endl;
        std::vector<double> potential_field;
        ros::Time current_time=ros::Time::now();
        double period=current_time.toSec()-previous_time.toSec();
        previous_time=current_time;
        double threshold=0.2;
        if(period>threshold)
            return;
        unsigned int aux_it;
        if(obstacles_positions_current.size()<=obstacles_positions_previous.size())
            aux_it=obstacles_positions_current.size();
        else
            aux_it=obstacles_positions_previous.size();
        std::vector<Eigen::Vector3d> current_v;
        current_v.resize(aux_it);
        contour_data_msg.obstacles_velocities.twist_array.clear();
        for(int i=0; i<aux_it; ++i)
        {
            current_v[i]=(obstacles_positions_current[i]-obstacles_positions_previous[i])/period;
            double velocity_sign=1.0;
            if(current_v[i].dot(Eigen::Vector3d::UnitX())<0)
                velocity_sign=-1.0; // Moving away from the obstacle
            potential_field.push_back(getPotentialPoint(obstacles_positions_current[i].norm(),velocity_sign*current_v[i].norm(), a_max, gain));
        }
        std::cout << "potential callback ..." << std::endl;
        std::vector<Eigen::Vector3d> force_field;
        std::vector<Eigen::Vector3d> risk_vectors;
        if(potential_field.size()<=previous_potential_field.size())
            aux_it=potential_field.size();
        else
            aux_it=previous_potential_field.size();

        for(int i=0; i<aux_it; ++i)
        {
            double force_magnitude=(potential_field[i]-previous_potential_field[i])/period; // Gradient of the potential field
            risk_vectors.push_back(potential_field[i]*(obstacles_positions_current[i].normalized())) ;
            force_field.push_back(force_magnitude*(obstacles_positions_current[i].normalized()));
        }
        std::cout << "potential callback start###" << std::endl;
        resulting_force=Eigen::Vector3d(0.0,0.0,0.0);
        resulting_risk_vector=Eigen::Vector3d(0.0,0.0,0.0);

        for(int i=0; i<risk_vectors.size(); ++i)
        {
            resulting_force+=force_field[i];
            resulting_risk_vector+=risk_vectors[i];
        }
        resulting_risk_vector = resulting_risk_vector / risk_vectors.size() ;

        std::cout << "resulting_risk_vector " << std::endl ;
        std::cout << "resulting_risk_vector.x() " << resulting_risk_vector.x() <<std::endl ;
        std::cout << "resulting_risk_vector.y() " << resulting_risk_vector.y() <<std::endl ;
        std::cout << "resulting_risk_vector.z() " << resulting_risk_vector.z() <<std::endl ;


        visualization_msgs::MarkerArray marker_array;
        marker_array=rviz_arrows(risk_vectors, obstacles_positions_current, std::string("potential_field"));
        visualization_msgs::Marker marker=rviz_arrow(resulting_risk_vector, Eigen::Vector3d(0,0,0), 0, std::string("resulting_risk_vector"));
        marker_array.markers.push_back(marker);
        visualization_markers_pub.publish(marker_array);
        previous_potential_field=potential_field;
    }


    double getPotentialPoint(const double & d, const double & v_i, const double & a_max, const double & gain)
    {

        float dstop = (v_i*v_i) / (2*a_max ) ;
        float dres ;
        if ( v_i <= 0 ) dres = d + dstop ;
        else dres = d - dstop ;

        if ( dres <= 0 || (1+v_i)/dres >= 1/gain)
            return  1.0;
        else if ( (1+v_i)/dres <= 0)
            return 0 ;
        else
            return gain *(1+v_i)/ dres;
    }

private:
    // ROS
    ros::NodeHandle n;
    ros::Subscriber obstacle_readings_sub;
    ros::Publisher visualization_markers_pub;
    //ros::Subscriber button_sub ;
    ros::Publisher feedback_pub ;
    std::string pose_topic_name;
    std::string sonar_topic_name;
    std::string velocity_cmd_topic_name;
    // ros::Publisher force_out;

    // Parameters
    double a_max;
    double ro;
    double freq;
    double robot_mass;
    double robot_radius;
    double laser_max_distance;
    double laser_min_distance;

    Eigen::Vector3d kp, kd;
    Eigen::Matrix3d kp_mat, kd_mat;

    // Helper variables
    bool init_flag;

    std::vector<Eigen::Vector3d> obstacles_positions_current;
    std::vector<Eigen::Vector3d> obstacles_positions_previous;

    Eigen::Vector3d resulting_force;
    Eigen::Vector3d resulting_risk_vector ;

    //laser
    laser_geometry::LaserProjection projector_;

    // listener
    tf::TransformListener listener_;


    // subscriber
    ros::Subscriber scan_sub_;


    void paramsCallback(navigation::potential_fieldConfig &config, uint32_t level)
    { gain = config.gain ;}


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
        // marker.header.frame_id = "/Pioneer3AT/base_link"; // for pioneer
        marker.header.frame_id = "laser0_frame";

        //marker.header.stamp = ros::Time::now();
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
        double pf_Start = ros::Time::now().toSec();
        std::cout << "I was called : (ms)" << (pf_Start - lastTimeCalled)*1000 << std::endl ;
        lastTimeCalled = pf_Start;
        std::cout << "sonar callback start **** " << std::endl;
        int counter = 0 ;

        obstacles_positions_current.clear();
        for(int i=0; i< msg->points.size(); ++i)
        {
            Eigen::Vector3d obstacle(msg->points[i].x,msg->points[i].y,msg->points[i].z);
            std::cout << "laser_max_distanse" << laser_max_distance << std::endl ;
            std::cout << "obstacle.norm" << obstacle.norm() << std::endl ;
            if(obstacle.norm()<laser_max_distance-0.01 && obstacle.norm()>laser_min_distance+0.01)
            {
                counter = counter +1 ;
                std::cout << " filling the obstacles " << counter <<  std::endl ;
                obstacles_positions_current.push_back(obstacle);
            }
        }

        if(!init_flag)
        {
            ROS_INFO("HELLO");
            init_flag=true;
            obstacles_positions_previous=obstacles_positions_current;
            return;
        }

        if(obstacles_positions_current.size()>0)
        {
            computePotentialField();
        }
        else
            std::cout << " NO CALL FOR POTENTIAL FIELD " << std::endl ;
        //if(linear_button_pressed)
        feedbackMaster();

        obstacles_positions_previous=obstacles_positions_current;
        std::cout << "sonar callback end ***" << std::endl;

    }


    void feedbackMaster()
    {
        // WEIRD MAPPING!!!

        //  phantom_omni::OmniFeedback force_feedback;
        //   force_feedback.force.x=resulting_risk_vector.y();
        //   force_feedback.force.y=resulting_risk_vector.z();
        //   force_feedback.force.z=resulting_risk_vector.x();

        // force_feedback.force.x=resulting_force.y();
        // force_feedback.force.y=resulting_force.z();
        //  force_feedback.force.z=resulting_force.x();
        //   force_out.publish(force_feedback);

        geometry_msgs::PoseStamped msg ;
        msg.header.stamp =  ros::Time::now();
        msg.pose.position.x=resulting_risk_vector.y() ;
        msg.pose.position.y =resulting_risk_vector.z() ;
        msg.pose.position.z=resulting_risk_vector.x() ;
        feedback_pub.publish(msg);

    }

    // HAPTIC BUTTON
    //    void buttonCallback(const phantom_omni::PhantomButtonEvent::ConstPtr& button)
    //    {
    //        if(button->grey_button==1)
    //            linear_button_pressed=true;
    //        else
    //            linear_button_pressed=false;


    //    }
};

int main(int argc, char **argv)
{

    std:: cout << " MAIN " << std::endl ;
    ros::init(argc, argv, "potential_field");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");

    // parameters
    double a_max;
    double ro;
    double freq;
    double robot_mass;
    double robot_radius;
    double gain;

    double kp_x;
    double kp_y;
    double kp_z;
    double kd_x;
    double kd_y;
    double kd_z;

    // Control gains
    n.param<double>("/potential_field/gain", kp_x, 1.0);
    n_priv.param<double>("Kp_y", kp_y, 1.0);
    n_priv.param<double>("Kp_z", kp_z, 1.0);
    n_priv.param<double>("Kp_x", kd_x, 1.0);
    n_priv.param<double>("Kp_y", kd_y, 1.0);
    n_priv.param<double>("Kp_z", kd_z, 1.0);

    Eigen::Vector3d kp(kp_x,kp_y,kp_z);
    Eigen::Vector3d kd(kd_x,kd_y,kd_z);

    double laser_max_distance;
    double laser_min_distance;


    //initialize operational parameters
    n_priv.param<double>("laser_max_distance", laser_max_distance,2.0);
    n_priv.param<double>("laser_min_distance", laser_min_distance,0.2);

    n_priv.param<double>("ro", ro, 3.0);
    n_priv.param<double>("frequency", freq, 50.0);
    n_priv.param<double>("acc_max", a_max, 1.0);
    n_priv.param<double>("robot_mass", robot_mass, 1.0);
    n_priv.param<double>("robot_radius", robot_radius, 0.2);
    n_priv.param<double>("gain", gain, 1.0);
    ros::Rate loop_rate(freq);
    std::string pose_topic_name = "/pose";
    std::string sonar_topic_name = "/RosAria/sonar";
    ForceField potential_field(n, freq, ro,gain, kp, kd, laser_min_distance, laser_max_distance, robot_mass, robot_radius, pose_topic_name, sonar_topic_name);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

