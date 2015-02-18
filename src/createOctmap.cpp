#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
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
#include <haptic_teleoperation/hmm_srv.h>
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


using namespace std;
using namespace fcl;
//using namespace octomap;

Eigen::Vector3d robotpose ;
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
    //message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    //tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    //ros::Publisher laser_pub;
    ros::Subscriber laser_sub;
    //ros::Publisher visualization_markers_pub_map ;
    ros::Publisher octmap_pub;
    ros::Publisher collide_F_pub ;
    ros::Subscriber slave_pose_sub ;


    LaserScanToPointCloud(ros::NodeHandle n):n_(n)
    {

        laser_sub = n_.subscribe("/scan",1, &LaserScanToPointCloud::laserCallback, this);
        slave_pose_sub = n_.subscribe("/RosAria/pose" , 100 ,&LaserScanToPointCloud::poseCallback, this);

        //laser_pub = n_.advertise<sensor_msgs::PointCloud2>("pointCloudObs", 10);
        //  visualization_markers_pub_map = n_.advertise<visualization_msgs::MarkerArray>("risk_vector_marker", 1);
        octmap_pub = n_.advertise<octomap_msgs::Octomap>("octomap_rviz", 1);
        collide_F_pub = n_.advertise<std_msgs::Bool>("collision_flag",1) ;


    }

    void laserCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        //std::cout<<"LASER" << scan_in->header.frame_id << std::endl ;
        if(!listener_.waitForTransform(scan_in->header.frame_id,
                                       "base_link",
                                       // "Pioneer3AT/base_link",  // GAZEBO
                                       //ros::Time::now(),
                                       scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                                       ros::Duration(3.0))){
            std::cout << "RETURN" << std::endl ;
            return;
        }
        sensor_msgs::PointCloud msg;
        //projector_.projectLaser(*scan_in, msg);
        projector_.transformLaserScanToPointCloud("base_link",*scan_in, msg,listener_);
        // std::cout << "cloud_size" << msg.points.size() << std::endl;
        // std::cout << "cloud_0" << msg.points[0].x << std::endl;



        octomap::OcTree* st_tree = new octomap::OcTree(0.1);
        //     OcTree st_tree (0.01); // create empty tree with resolution 0.1
        octomap::Pointcloud st_cld;
        // visualization_msgs::MarkerArray marker_array ; // = rviz_arrows(force_field, obstacles_positions_current, std::string("potential_field"));
        for(int i = 0;i<msg.points.size();i++){
            octomap::point3d endpoint((float) msg.points[i].x,(float) msg.points[i].y,(float) msg.points[i].z);
            //  visualization_msgs::Marker marker =  rviz_arrow(endpoint,i ,"tree");
            //    marker_array.markers.push_back(marker);
            st_cld.push_back(endpoint);

        }
        //visualization_markers_pub_map.publish(marker_array);

        // maybe this should be th position of the robot

        octomap::point3d origin(0.0,0.0,0.0);
        st_tree->insertPointCloud(st_cld,origin);
        st_tree->updateInnerOccupancy();
        st_tree->writeBinary("static_occ.bt");
        OcTree* st_tree2 = new OcTree(boost::shared_ptr<const octomap::OcTree>(st_tree));

        octomap_msgs::Octomap octomap ;
        octomap.binary = 1 ;
        octomap.id = 1 ;
        octomap.resolution =0.05 ;
        octomap.header.frame_id = "/map";
        octomap.header.stamp = ros::Time::now();
        bool res = octomap_msgs::fullMapToMsg(*st_tree, octomap);
        //octomap.data = td_vector_to_py_list(octomap.data)  ;
        octmap_pub.publish(octomap) ;


        std_msgs::Bool collide_flag;



        boost::shared_ptr<Sphere> Shpere0(new Sphere(0.5));
//        GJKSolver_libccd solver;
//        Vec3f contact_points;
//        FCL_REAL penetration_depth;
//        Vec3f normal;

        Transform3f tf0, tf1;
        tf0.setIdentity();
        // is this the postion in x,y and x
        tf0.setTranslation(Vec3f(robotpose(0),robotpose(1),robotpose(2)));
        // should I add the ones from the msg ????
        tf0.setQuatRotation(Quaternion3f(0, 0, 0, 0));

        tf1.setIdentity();

        // CollisionObject tree_obj((boost::shared_ptr<CollisionGeometry>(st_tree)));

        // convert the octomap::octree to fcl::octree fcl_octree object
        CollisionObject co0(Shpere0, tf0);
        std::vector<CollisionObject*> boxes;
        generateBoxesFromOctomap(boxes, *st_tree2);


        for(size_t i = 0; i < boxes.size(); ++i)
        {

            //bool res2 = solver.shapeIntersect(*Shpere0, tf0, boxes[i], tf1, &contact_points, &penetration_depth, &normal);
           // std::cout << "res2" << res2 << std::endl ;
            static const int num_max_contacts = 1000;//std::numeric_limits<int>::max();
            std::cout << "num_max_contacts: " <<  num_max_contacts << std::endl ;
            static const bool enable_contact = false;
            fcl::CollisionResult result;
            fcl::CollisionRequest request(num_max_contacts, enable_contact);
            fcl::collide(&co0, boxes[i], request, result);
            vector<Contact> contacts;

            result.getContacts(contacts);
            std::cout << "size" <<  contacts.size() << std::endl ;
            if ( contacts.size() > 0 )
           {

               collide_flag.data = true ;
               collide_F_pub.publish(collide_flag) ;

           }
           else
           { collide_flag.data = false ;
               collide_F_pub.publish(collide_flag) ;
           }



            //OR
            /*
            if (res2 == 1 )
            {
                collide_F_pub.publish(collide_flag) ;
                exit(1);
            }*/
        }
        //                collide_flag.data = res2 ;
        //                collide_F_pub.publish(collide_flag) ;
        for(size_t i = 0; i < boxes.size(); ++i)
            delete boxes[i];


    }
    void poseCallback(const nav_msgs::Odometry::ConstPtr & robot_pose)
    {
        // std::cout << "get robot data " << std::endl ;
        robotpose(0) =  robot_pose->pose.pose.position.x ;
        robotpose(1) =  robot_pose->pose.pose.position.y  ;
        robotpose(2) =  robot_pose->pose.pose.position.z ;
        poseQ[0] = robot_pose->pose.pose.orientation.x;
        poseQ[1] = robot_pose->pose.pose.orientation.y;
        poseQ[2] = robot_pose->pose.pose.orientation.z;
        poseQ[3] = robot_pose->pose.pose.orientation.w;
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
        std::cout << "boxes size: " << boxes.size() << std::endl;
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
    LaserScanToPointCloud lstopc(n);
    std::cout << "Object created" << std::endl ;
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
