#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/PoseStamped.h"

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
Eigen::Vector3d robotvel ;

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



class LaserScanToPointCloud{
public:

    ros::NodeHandle n_;

    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;

    ros::Subscriber laser_sub;
    ros::Subscriber slave_pose_sub ;
    ros::Subscriber slave_vel_sub;

    ros::Publisher vis_pub ;
    ros::Publisher  vis_cube_pub ;
    ros::Publisher octmap_pub;
    ros::Publisher collide_F_pub ;

    double roll, pitch, yaw ;

    LaserScanToPointCloud(ros::NodeHandle n):n_(n)
    {

        std::cout << "Object 2" << std::endl ;

        // subscribers
        laser_sub = n_.subscribe("/iris/iris/laser/scan",10, &LaserScanToPointCloud::laserCallback, this);
        slave_pose_sub = n_.subscribe("iris/mavros/vision_pose/pose" , 100 ,&LaserScanToPointCloud::poseCallback, this);
        slave_vel_sub = n_.subscribe("/uav/cmd_vel" , 100 ,&LaserScanToPointCloud::velCallback, this);

        // Publishers
        vis_pub = n_.advertise<visualization_msgs::Marker>("Sphere", 1);
        vis_cube_pub = n_.advertise<visualization_msgs::MarkerArray>("Cube", 1);
        octmap_pub = n_.advertise<octomap_msgs::Octomap>("octomap_rviz", 1);
        collide_F_pub = n_.advertise<std_msgs::Bool>("collision_flag",1) ;



    }

    void laserCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        double tstart= ros::Time::now().toSec() ;
        if(!listener_.waitForTransform(scan_in->header.frame_id,
                                       "world",
                                       scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                                       ros::Duration(1.0)))
        {
            std::cout << "RETURN" << std::endl ;
            return;
        }

        sensor_msgs::PointCloud msg;
        //projector_.projectLaser(*scan_in, msg);
        projector_.transformLaserScanToPointCloud("world",*scan_in, msg,listener_);

        // create empty tree with resolution 0.1
        octomap::OcTree* st_tree = new octomap::OcTree(0.1);
        octomap::Pointcloud st_cld;
        double t01 = ros::Time::now().toSec();

        // filtring the laser data
        for(int i = 0;i<msg.points.size();i++){
            if( sqrt( ((msg.points[i].x - robotpose(0))*(msg.points[i].x-robotpose(0)))
                      + ((msg.points[i].y-robotpose(1))*(msg.points[i].y-robotpose(1)))
                      + ((msg.points[i].z-robotpose(2))*(msg.points[i].z-robotpose(2)))) > 0.5
                    &&
                    sqrt( ((msg.points[i].x-robotpose(0))*(msg.points[i].x-robotpose(0)))
                          + ((msg.points[i].y-robotpose(1))*(msg.points[i].y-robotpose(1)))
                          + (msg.points[i].z-robotpose(2))*(msg.points[i].z-robotpose(2))) < 4)
            {
                octomap::point3d endpoint((float) msg.points[i].x,(float) msg.points[i].y,(float) msg.points[i].z);
                st_cld.push_back(endpoint);
            }
           // std::cout << "No Obstacles are recorded  " << std::endl ;
        }
        
        double t10 = ros::Time::now().toSec();
        std::cout << "Operation Time for getting the distance between the robot and the obstcale: " << t10- t01 << std::endl ;

        // maybe this should be the position of the robot
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
        boost::shared_ptr<Box> Shpere0(new Box(0.7,0.7,0.7));
        //        GJKSolver_libccd solver;
        //        Vec3f contact_points;
        //        FCL_REAL penetration_depth;
        //        Vec3f normal;


        visualization_msgs::MarkerArray marker_array ;
        visualization_msgs::Marker marker ;


        Transform3f tf0;
        tf0.setIdentity();
        double deltaT = 4;
        double x = robotpose(0) ;
        double y = robotpose(1) ;
        double speedInx = robotvel(0) ; // * sin(theta) ;
        double speedIny = robotvel(1) ; //* cos(theta);
        double xPrime = speedInx*deltaT ;
        double yPrime = speedIny*deltaT ;
        std::cout << "Yaw " << yaw *180/M_PI  << std::endl ;
        // the next step of the robot not the actual position
        tf0.setTranslation(Vec3f(robotpose(0) +xPrime, robotpose(1) +yPrime,robotpose(2)));
        marker = drawCUBE(Vec3f(robotpose(0) +xPrime, robotpose(1) +yPrime,robotpose(2)),1000,3) ;

        marker_array.markers.push_back(marker);


        CollisionObject co0(Shpere0, tf0);
        // for visualization
        AABB a = co0.getAABB() ;
        Vec3f vec =  a.center() ;
        drawSphere( vec ) ;

        double t1 = ros::Time::now().toSec();

        std::vector<CollisionObject*> boxes;
        generateBoxesFromOctomap(boxes, *st_tree2);

	bool collisionDetected = false;
        for(size_t i = 0; i < boxes.size(); ++i)
        {
            CollisionObject* box =  boxes[i];
            static const int num_max_contacts = std::numeric_limits<int>::max();
            static const bool enable_contact = true ;

            // for visualization
            AABB b = box->getAABB() ;
            Vec3f vec2 =  b.center();

            fcl::CollisionResult result;
            fcl::CollisionRequest request(num_max_contacts, enable_contact);
            fcl::collide(&co0, box, request, result);

            if (result.isCollision() == true )
            {

                marker = drawCUBE(vec2,i,2) ;
                marker_array.markers.push_back(marker);
                //std::cout << "inCollision " << std::endl ;
                // exit(0) ;
                collide_flag.data = true ;
                collide_F_pub.publish(collide_flag) ;
		collisionDetected = true;

            }
            else
            {
                marker = drawCUBE(vec2, i, 1) ;
                marker_array.markers.push_back(marker);
            }

        }
        vis_cube_pub.publish(marker_array);
        for(size_t i = 0; i < boxes.size(); ++i)
            delete boxes[i];
        double t2 = ros::Time::now().toSec();
	if(!collisionDetected)
	{
	  collide_flag.data = false ;
          collide_F_pub.publish(collide_flag) ;	  
	}
      //  std::cout << "Operation Time is: " << (t2 - t1)*1000.0f << std::endl ;
      //  std::cout << "time or call back function: " << (t2 - tstart)*1000.0f << std::endl ;
    }
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & robot_pose)
    {
        // std::cout << "get robot data " << std::endl ;
        robotpose(0) =  robot_pose->pose.position.x ;
        robotpose(1) =  robot_pose->pose.position.y  ;
        robotpose(2) =  robot_pose->pose.position.z ;
        poseQ[0] = robot_pose->pose.orientation.x;
        poseQ[1] = robot_pose->pose.orientation.y;
        poseQ[2] = robot_pose->pose.orientation.z;
        poseQ[3] = robot_pose->pose.orientation.w;
        tf::Quaternion q(poseQ[0], poseQ[1] ,poseQ[2],poseQ[3]);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::cout << "Yaw  from callback" << yaw *180/M_PI  << std::endl ;

    }

    void velCallback (const geometry_msgs::TwistStamped::ConstPtr&  cmd_vel)
    {
        robotvel(0) =  cmd_vel->twist.linear.x ;
        robotvel(1) =  cmd_vel->twist.linear.y  ;
        robotvel(2) =  cmd_vel->twist.linear.z ;
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
        marker.header.frame_id = "/uav/baselink_ENU";
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
        vis_pub.publish( marker );
    }

    visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color)
    {
        visualization_msgs::Marker marker;
        //        marker.header.frame_id = "odom";
        marker.header.frame_id = "/world";

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
            marker.color.g = 0.0;

        }
        else if(c_color == 2)
        {
            marker.color.r = 1.0;
            marker.color.b = 0.0;
            marker.color.g = 0.0;

        }
        else
        {
            marker.color.r = 0.0;
            marker.color.b = 0.0;
            marker.color.g = 1.0;

        }

        marker.lifetime = ros::Duration(0.3);
        //only if using a MESH_RESOURCE marker type:
        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        return marker ;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Collision_Detection_node");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    double freq;
    n_priv.param<double>("frequency", freq, 100.0);
    ros::Rate loop_rate(freq);
    LaserScanToPointCloud lstopc(n);
    std::cout << "Object created" << std::endl ;
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
