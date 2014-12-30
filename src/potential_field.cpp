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
#include <haptic_teleoperation/TwistArray.h>
#include <haptic_teleoperation/ContourData.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <haptic_teleoperation/potential_fieldConfig.h>
#include <phantom_omni/PhantomButtonEvent.h>
#include <algorithm>


haptic_teleoperation::ContourData contour_data_msg;
const double PI=3.14159265359;
#define BILLION 1000000000
double lastTimeCalled ;//= ros::Time::now().toSec();
double lastDataUpdate;
bool haltControl= false;
class ForceField
{
public:
    dynamic_reconfigure::Server<haptic_teleoperation::potential_fieldConfig> param_server;
    dynamic_reconfigure::Server<haptic_teleoperation::potential_fieldConfig>::CallbackType param_callback_type;
    std::vector<double> previous_potential_field;
    ros::Time previous_time;

    ForceField(ros::NodeHandle & n_,
               double & freq_,
               double & gain_,
               double & laser_min_distance_,
               double & laser_max_distance_,
               std::string & sonar_topic_name_)
        : n(n_),
          freq(freq_),
          gain(gain_),
          laser_min_distance(laser_min_distance_),
          laser_max_distance(laser_max_distance_),
          sonar_topic_name(sonar_topic_name_)
    {
        a_max=1.0;
        param_callback_type = boost::bind(&ForceField::paramsCallback, this, _1, _2);
        param_server.setCallback(param_callback_type);

        visualization_markers_pub = n.advertise<visualization_msgs::MarkerArray>("risk_vector_marker", 1);

        feedback_pub = n.advertise<geometry_msgs::PoseStamped>("pf_force_feedback", 1);

        init_flag=false;
        obstacle_readings_sub = n.subscribe("/cloud",100, &ForceField::sonarCallback, this);
        lastTimeCalled = ros::Time::now().toSec();

    };


    /* This function compute the potential field for each point and it sums all the points, then it takes the gradiant.
    It is only going to be called when the robot sence the exiatance of the obstacle */

    void computePotentialField()
    {

        std::cout<< "compute Potential field ##############3" << std::endl;

        // Compute current robot velocity based on odometry readings
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
        std::cout<< "size of aux &&&&&&&& " << aux_it << std::endl;



        std::vector<Eigen::Vector3d> current_v;
        current_v.resize(aux_it);


        for(int i=0; i<aux_it; ++i)
        {

            std::cout<< "compute velocity  ##############3" << std::endl;

            current_v[i]=(obstacles_positions_current[i]-obstacles_positions_previous[i])/period;
            double velocity_sign=1.0;
            if(current_v[i].dot(Eigen::Vector3d::UnitX())<0)
                velocity_sign=-1.0; // Moving away from the obstacle

            potential_field.push_back(getPotentialPoint(obstacles_positions_current[i].norm(),velocity_sign*current_v[i].norm(), a_max, gain));
        }
        std::vector<Eigen::Vector3d> force_field;
        std::vector<Eigen::Vector3d> risk_vectors;
        if(potential_field.size()<=previous_potential_field.size())
            aux_it=potential_field.size();
        else
            aux_it=previous_potential_field.size();



        for(int i=0; i<aux_it; ++i)
        {

            std::cout<< "finidng the force @@@@@@@@@@@@@" << std::endl;

            double force_magnitude=(potential_field[i]-previous_potential_field[i])/period; // Gradient of the potential field
            risk_vectors.push_back(potential_field[i]*(obstacles_positions_current[i].normalized())) ;
            force_field.push_back(force_magnitude*(obstacles_positions_current[i].normalized()));
        }

        resulting_force=Eigen::Vector3d(0.0,0.0,0.0);
        resulting_risk_vector=Eigen::Vector3d(0.0,0.0,0.0);


        //        double min = 1.0 ;
        //        double max = 0.0 ;
        //        int index1 =0;
        //        int index2 =0;

        if ( aux_it != 0.0 )
        {
           std::cout<< "if" << std::endl;

            for(int i=0; i<risk_vectors.size(); ++i)
            {
                resulting_risk_vector += risk_vectors[i] ;
                resulting_force+=force_field[i];

                //                double d = risk_vectors[i].norm() ;
                //                if (d > max)
                //                {
                //                    max= d ;
                //                    index1 = i ;

                //                }
                //                else if (d < min)
                //                {
                //                    min= d ;
                //                    index2 = i ;
                //                    std::cout << "min" << min <<std::endl;

                //                }


            }
            //resulting_force = resulting_force / aux_it ;
         //  resulting_risk_vector = resulting_risk_vector / aux_it ;
            // summation  // limited to 1
            //resulting_risk_vector = resulting_risk_vector  ;
            //resulting_force = resulting_force ;
            // mean
            // resulting_risk_vector = resulting_risk_vector / risk_vectors.size()  ;
            // resulting_force = resulting_force/ risk_vectors.size() ;
            // min , max // limites to 1
            //resulting_risk_vector = risk_vectors[index1] + risk_vectors[index2];
            // resulting_risk_vector = resulting_risk_vector / 2.0 ;
            //resulting_force=force_field[index1] + force_field[index2];
            //resulting_force=resulting_force /2.0;
        }
        else
        {
            std::cout<< "else" << std::endl;

            resulting_risk_vector=Eigen::Vector3d(0.0,0.0,0.0);
            resulting_force=Eigen::Vector3d(0.0,0.0,0.0);

        }




        visualization_msgs::MarkerArray marker_array= rviz_arrows(risk_vectors, obstacles_positions_current, std::string("potential_field"));
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
        {
            //std::cout << "risk vector" << gain *(1+v_i)/ dres  << std::endl ;
            return gain *(1+v_i)/ dres;
        }





    }

private:
    // ROS
    ros::NodeHandle n;

    // subscriber
    ros::Subscriber obstacle_readings_sub;

    // publisher
    ros::Publisher visualization_markers_pub;
    ros::Publisher feedback_pub ;

    std::string sonar_topic_name;

    // Parameters
    double a_max;
    double freq;
    double gain ;
    double laser_max_distance;
    double laser_min_distance;

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



    void paramsCallback(haptic_teleoperation::potential_fieldConfig &config, uint32_t level)
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
       // marker.header.frame_id = "laser0_frame";
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
        ros::Duration d(0.1);
        marker.lifetime = d ;

        marker.header.stamp = ros::Time::now() ;
        return marker;
    }

    visualization_msgs::MarkerArray rviz_arrows(const std::vector<Eigen::Vector3d> & arrows, const std::vector<Eigen::Vector3d> arrows_origins, std::string name_space)
    {
        visualization_msgs::MarkerArray marker_array;
        for(int i=0; i< arrows.size();i=i+30)
        {
            marker_array.markers.push_back(rviz_arrow(arrows[i], arrows_origins[i], (i+1), name_space));

        }
        return marker_array;
    }
    void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
    {
        // data update
        lastDataUpdate = ros::Time::now().toSec();

        double pf_Start = ros::Time::now().toSec();
        std::cout << "I was called : (ms)" << (pf_Start - lastTimeCalled)*1000 << std::endl ;

        lastTimeCalled = pf_Start;
        int counter = 0 ;

        obstacles_positions_current.clear();
        for(int i=0; i< msg->points.size(); ++i)
        {
            //std::cout << "Collecting obstacles" << std::endl ;

            Eigen::Vector3d obstacle(msg->points[i].x,msg->points[i].y,msg->points[i].z);

            if(obstacle.norm()<5.0 && obstacle.norm()>3.0)
            {
                std::cout << "push obstacles **********************" << std::endl ;

                counter = counter +1 ;
                obstacles_positions_current.push_back(obstacle);
            }
        }

        if(!init_flag)
        {
            init_flag=true;
            obstacles_positions_previous=obstacles_positions_current;
            return;
        }


        //if(obstacles_positions_current.size()>0 )//&& !haltControl
        // {
        computePotentialField();
        // }
        feedbackMaster();

        obstacles_positions_previous=obstacles_positions_current;
        return;

    }


    void feedbackMaster()
    {

        //  phantom_omni::OmniFeedback force_feedback;
        //   force_feedback.force.x=resulting_risk_vector.y();
        //   force_feedback.force.y=resulting_risk_vector.z();
        //   force_feedback.force.z=resulting_risk_vector.x();

        // force_feedback.force.x=resulting_force.y();
        // force_feedback.force.y=resulting_force.z();
        //  force_feedback.force.z=resulting_force.x();
        //   force_out.publish(force_feedback);

        //   std::cout << "resulting_risk_vector" << resulting_risk_vector << endl ;
        geometry_msgs::PoseStamped msg ;

        msg.header.stamp =  ros::Time::now();
        // reflecting the potential field itself
        // msg.pose.position.x=-resulting_risk_vector.x() ;
       //  msg.pose.position.y =resulting_risk_vector.y() ;
       //  msg.pose.position.z=resulting_risk_vector.z() ;

        // reflecting the gradiant of the potential field
       msg.pose.position.x=-resulting_force.x() ;
       msg.pose.position.y =resulting_force.y() ;
       msg.pose.position.z=resulting_force.z() ;

         std::cout << "resulting_risk_vector.x() " << msg.pose.position.x <<std::endl ;
         std::cout << "resulting_risk_vector.y() " << msg.pose.position.y <<std::endl ;
          std::cout << "resulting_risk_vector.z() " <<  msg.pose.position.z <<std::endl ;

        feedback_pub.publish(msg);
    }

};

int main(int argc, char **argv)
{

    //std:: cout << " MAIN " << std::endl ;
    ros::init(argc, argv, "potential_field");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");

    // parameters
    double a_max;
    double freq;
    double gain;

    double laser_max_distance;
    double laser_min_distance;


    //initialize operational parameters
    n_priv.param<double>("laser_max_distance", laser_max_distance,4.0);
    n_priv.param<double>("laser_min_distance", laser_min_distance,0.2);

    n_priv.param<double>("frequency", freq, 50.0);
    n_priv.param<double>("acc_max", a_max, 1.0);
    n_priv.param<double>("gain", gain, 0.1);
    ros::Rate loop_rate(freq);
    std::string sonar_topic_name = "/cloud";
    double periodThreshold = 50/1000.0f;
    ForceField potential_field(n, freq, gain,laser_min_distance, laser_max_distance,sonar_topic_name);

    while(ros::ok())
    {
        if( (ros::Time::now().toSec() - lastDataUpdate) > periodThreshold)
        {
            haltControl = true;
        }
        else
        {
            haltControl = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

