#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/filter.h>
#include <octomap_msgs/conversions.h>
#include <deque>
//#include <ghmm/GHMM.hpp>
//#include <haptic_teleoperation/Num.h>
//#include <haptic_teleoperation/hmm_srv.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>

#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/ccd/motion.h"
#include <stdlib.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <octomap/octomap.h>
#include "fcl/BV/AABB.h"
#include "fcl/collision_object.h"
//#include <moveit_visual_tools/moveit_visual_tools.h>



using namespace std;
using namespace fcl;
//using namespace octomap;

Eigen::Vector3d robotpose ;
Eigen::Vector3d robotvel;

double poseQ[4] ;

FCL_REAL extents [6] = {0, 0, 0, 10, 10, 10};
GJKSolver_libccd solver1;
GJKSolver_indep solver2;
//#define BOOST_CHECK_FALSE(p) BOOST_CHECK(!(p))
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


const float res = 0.1;
std::deque< pcl::PointCloud<pcl::PointXYZ> > cloud_seq_loaded;



struct TStruct
{
    std::vector<double> records;
    double overall_time;
    TStruct() { overall_time = 0; }
    void push_back(double t)
    {
        records.push_back(t);
        overall_time += t;
    }
};



class collisionDetection_GV{
public:

    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    //message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    //tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    //ros::Publisher laser_pub;
    ros::Subscriber laser_sub;
    ros::Publisher vis_pub ;
    ros::Publisher octmap_pub;
    ros::Publisher collide_F_pub ;
    ros::Subscriber slave_pose_sub ;
    ros::Publisher  vis_cude_pub ;
    ros::Subscriber slave_vel_sub ;


    double roll , pitch, yaw ;
    //  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

    collisionDetection_GV(ros::NodeHandle n):n_(n)
    {

        laser_sub = n_.subscribe("/scan",1, &collisionDetection_GV::laserCallback, this);
        slave_pose_sub = n_.subscribe("/RosAria/pose" , 100 ,&collisionDetection_GV::poseCallback, this);
        slave_vel_sub = n_.subscribe("/RosAria/cmd_vel" , 100 ,&collisionDetection_GV::velCallback, this);

        //slave_pose_sub = n_.subscribe("/mavros/vision_pose/pose" , 100 ,&collisionDetection_GV::poseCallback, this);

        //laser_pub = n_.advertise<sensor_msgs::PointCloud2>("pointCloudObs", 10);
        vis_pub = n_.advertise<visualization_msgs::Marker>("Sphere", 1);
        vis_cude_pub = n_.advertise<visualization_msgs::MarkerArray>("Cube", 1);
        octmap_pub = n_.advertise<octomap_msgs::Octomap>("octomap_rviz", 1);
        collide_F_pub = n_.advertise<std_msgs::Bool>("collision_flag",1) ;

        //      visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link","/moveit_visual_markers"));

    }

    void velCallback (const geometry_msgs::Twist::ConstPtr&  cmd_vel)
    {

        //std::cout << " GET VELOCITY " << std::endl ;
        robotvel(0) =  cmd_vel->linear.x ;
        robotvel(1) =  cmd_vel->linear.y  ;
        robotvel(2) =  cmd_vel->linear.z ;
    }

    void laserCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        double tstart= ros::Time::now().toSec() ;
        //std::cout<<"LASER" << scan_in->header.frame_id << std::endl ;
        if(!listener_.waitForTransform(scan_in->header.frame_id,
                                       "odom",
                                       //"/uav/baselink_ENU",
                                       scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                                       ros::Duration(1.0)))
        {
            std::cout << "RETURN" << std::endl ;
            return;
        }

        sensor_msgs::PointCloud msg;
        //projector_.projectLaser(*scan_in, msg);
        //projector_.transformLaserScanToPointCloud("/uav/base_link_ENU",*scan_in, msg,listener_);

        projector_.transformLaserScanToPointCloud("odom",*scan_in, msg,listener_);



        octomap::OcTree* st_tree = new octomap::OcTree(0.1);
        //     OcTree st_tree (0.01); // create empty tree with resolution 0.1
        octomap::Pointcloud st_cld;
        double t01 = ros::Time::now().toSec();

        // visualization_msgs::MarkerArray marker_array ; // = rviz_arrows(force_field, obstacles_positions_current, std::string("potential_field"));
        for(int i = 0;i<msg.points.size();i++){
            if( sqrt( ((msg.points[i].x - robotpose(0))*(msg.points[i].x-robotpose(0)))
                      + ((msg.points[i].y-robotpose(1))*(msg.points[i].y-robotpose(1)))
                      + ((msg.points[i].z-robotpose(2))*(msg.points[i].z-robotpose(2)))) > 0.3
                    &&
                    sqrt( ((msg.points[i].x-robotpose(0))*(msg.points[i].x-robotpose(0)))
                          + ((msg.points[i].y-robotpose(1))*(msg.points[i].y-robotpose(1)))
                          + (msg.points[i].z-robotpose(2))*(msg.points[i].z-robotpose(2))) < 2)
            {
                octomap::point3d endpoint((float) msg.points[i].x,(float) msg.points[i].y,(float) msg.points[i].z);
                //  visualization_msgs::Marker marker =  rviz_arrow(endpoint,i ,"tree");
                //    marker_array.markers.push_back(marker);
                st_cld.push_back(endpoint);
            }
           // std::cout << "No Obstacles are recorded  " << std::endl ;


        }
        
        double t10 = ros::Time::now().toSec();
        //std::cout << "Operation Time for getting the distance between the robot and the obstcale: " << t10- t01 << std::endl ;

        //visualization_markers_pub_map.publish(marker_array);
        // maybe this should be th position of the robot
        octomap::point3d origin(0.0,0.0,0.0);
        st_tree->insertPointCloud(st_cld,origin);
        st_tree->updateInnerOccupancy();
        st_tree->writeBinary("static_occ.bt");
        // convert the octomap::octree to fcl::octree fcl_octree object
        OcTree* st_tree2 = new OcTree(boost::shared_ptr<const octomap::OcTree>(st_tree));

        octomap_msgs::Octomap octomap ;
        octomap.binary = 1 ;
        octomap.id = 1 ;
        octomap.resolution =0.1;
        octomap.header.frame_id = "/map";
        octomap.header.stamp = ros::Time::now();

        bool res = octomap_msgs::fullMapToMsg(*st_tree, octomap);
        octmap_pub.publish(octomap) ;


        std_msgs::Bool collide_flag;
        boost::shared_ptr<Box> Shpere0(new Box(1,1,1));
        //        GJKSolver_libccd solver;
        //        Vec3f contact_points;
        //        FCL_REAL penetration_depth;
        //        Vec3f normal;

        Transform3f tf0;
        tf0.setIdentity();
        // this should be changed ( i should subscribe for the commmanded velocity)


        double timeSample  = 0.5;
      //  double theta = yaw * 180/M_PI  ;

       // std::cout << "yaw in degrees  " << theta  << std::endl ;
        std::cout << " "  << std::endl ;
        std::cout << " "  << std::endl ;

        double x = robotpose(0) ;
        double y = robotpose(1) ;
        double speedInx = robotvel(0) ; // * sin(theta) ;
        double speedIny = robotvel(1) ; //* cos(theta);
        double xPrime = speedInx*cos(yaw)*timeSample ;
        double yPrime = speedInx*sin(yaw)*timeSample ;

        tf0.setTranslation(Vec3f(x+ xPrime,y+yPrime,robotpose(2)));
        CollisionObject co0(Shpere0, tf0);
        AABB a = co0.getAABB() ;
        Vec3f vec =  a.center() ;
        drawSphere( vec ) ;

        double t1 = ros::Time::now().toSec();

        std::vector<CollisionObject*> boxes;
        generateBoxesFromOctomap(boxes, *st_tree2);

        visualization_msgs::MarkerArray marker_array ;
        for(size_t i = 0; i < boxes.size(); ++i)
        {
            CollisionObject* box =  boxes[i];
            //bool res2 = solver.shapeIntersect(*Shpere0, tf0, box, tf1, &contact_points, &penetration_depth, &normal);
            //std::cout << "res2" << res2 << std::endl ;
            static const int num_max_contacts = std::numeric_limits<int>::max();//std::numeric_limits<int>::max();
            static const bool enable_contact = true ;

            AABB b = box->getAABB() ;
            Vec3f vec2 =  b.center();

            visualization_msgs::Marker marker ; // = drawCUBE(vec2, i) ;
            // marker_array.markers.push_back(marker);
            fcl::CollisionResult result;
            fcl::CollisionRequest request(num_max_contacts, enable_contact);
            fcl::collide(&co0, box, request, result);
            //          vector<Contact> contacts;
            //          result.getContacts(contacts);
            //          std::cout << "size: " <<  contacts.size() << std::endl ;
            if (result.isCollision() == true )
            {

                marker = drawCUBE(vec2,i,2) ;
                marker_array.markers.push_back(marker);
                std::cout << "inCollision " << std::endl ;
                // exit(0) ;
                collide_flag.data = true ;
                collide_F_pub.publish(collide_flag) ;
                //goto after_loop;                 ;
            }
            else
            {
                marker = drawCUBE(vec2, i, 1) ;
                marker_array.markers.push_back(marker);
                collide_flag.data = false ;
                collide_F_pub.publish(collide_flag) ;
            }
        }
       // after_loop:
        vis_cude_pub.publish(marker_array);
        for(size_t i = 0; i < boxes.size(); ++i)
            delete boxes[i];
        double t2 = ros::Time::now().toSec();
       // std::cout << "Operation Time is " << t2 - t1 << std::endl ;

      //  std::cout << "time or call back function" << t2 - tstart << std::endl ;
    }
    //void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & robot_pose)

    void poseCallback(const nav_msgs::Odometry::ConstPtr & robot_pose)
    {

        std::cout << "get ROBOT POSE  "  << std::endl ;

        //        // std::cout << "get robot data " << std::endl ;
        //        robotpose(0) =  robot_pose->pose.position.x ;
        //        robotpose(1) =  robot_pose->pose.position.y  ;
        //        robotpose(2) =  robot_pose->pose.position.z ;
        //        poseQ[0] = robot_pose->pose.orientation.x;
        //        poseQ[1] = robot_pose->pose.orientation.y;
        //        poseQ[2] = robot_pose->pose.orientation.z;
        //        poseQ[3] = robot_pose->pose.orientation.w;

        robotpose(0) =  robot_pose->pose.pose.position.x ;
        robotpose(1) =  robot_pose->pose.pose.position.y  ;
        robotpose(2) =  robot_pose->pose.pose.position.z ;

        poseQ[0] = robot_pose->pose.pose.orientation.x;
        poseQ[1] = robot_pose->pose.pose.orientation.y;
        poseQ[2] = robot_pose->pose.pose.orientation.z;
        poseQ[3] = robot_pose->pose.pose.orientation.w;
        std::cout << "get ROBOT POSE  "  << std::endl ;

        tf::Quaternion q(  poseQ[0] , poseQ[1] , poseQ[2] , poseQ[3]);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::cout << "roll " << roll  << std::endl ;
        std::cout << "pitch " << pitch  << std::endl ;

        std::cout << "yaw " << yaw  << std::endl ;

    }


    void generateBoxesFromOctomap(std::vector<CollisionObject*>& boxes, OcTree& tree)
    {

        std::vector<boost::array<FCL_REAL, 6> > boxes_ = tree.toBoxes();
        for(std::size_t i = 0; i < boxes_.size(); ++i)
        {
            FCL_REAL x = boxes_[i][0];
            FCL_REAL y = boxes_[i][1];
            FCL_REAL z = boxes_[i][2];
            FCL_REAL size = boxes_[i][3];
            FCL_REAL cost = boxes_[i][4];
            FCL_REAL threshold = boxes_[i][5];
            Box* box = new Box(size, size, size);
            box->cost_density = cost;
            box->threshold_occupied = threshold;
            CollisionObject* obj = new CollisionObject(boost::shared_ptr<CollisionGeometry>(box), Transform3f(Vec3f(x, y, z)));
            boxes.push_back(obj);
        }
        // std::cout << "boxes size: " << boxes.size() << std::endl;
    }


    void drawSphere(Vec3f vec )
    {
        visualization_msgs::Marker marker;
        //marker.header.frame_id = "odom";
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        // if(shape == "Sphere")
        marker.type = visualization_msgs::Marker::SPHERE;
        // if(shape == "Cube")
        //marker.type = visualization_msgs::Marker::CUBE;

        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = vec[0];
        marker.pose.position.y = vec[1];
        marker.pose.position.z = vec[2];
        marker.pose.orientation.x = poseQ[0];
        marker.pose.orientation.y = poseQ[1];
        marker.pose.orientation.z = poseQ[2];
        marker.pose.orientation.w = poseQ[3];
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(0.3);
        //only if using a MESH_RESOURCE marker type:
        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        vis_pub.publish( marker );
    }

    visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color)
    {
        visualization_msgs::Marker marker;
        //        marker.header.frame_id = "odom";
        marker.header.frame_id = "/odom";

        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        // if(shape == "Sphere")
        //marker.type = visualization_msgs::Marker::SPHERE;
        // if(shape == "Cube")
        marker.type = visualization_msgs::Marker::CUBE;

        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = vec[0];
        marker.pose.position.y = vec[1];
        marker.pose.position.z = vec[2];
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;//poseQ[1];
        marker.pose.orientation.z = 0;//poseQ[2];
        marker.pose.orientation.w = 0;//poseQ[3];

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        if(c_color == 1)
        {
            marker.color.r = 0.0;
            marker.color.b = 1.0;
        }
        else
        {
            marker.color.r = 1.0;
            marker.color.b = 0.0;
        }

        marker.lifetime = ros::Duration(0.3);
        //only if using a MESH_RESOURCE marker type:
        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        return marker ;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_scan_to_cloud");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    double freq;
    n_priv.param<double>("frequency", freq, 100.0);
    ros::Rate loop_rate(freq);
    collisionDetection_GV lstopc(n);
    std::cout << "Object created" << std::endl ;
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
