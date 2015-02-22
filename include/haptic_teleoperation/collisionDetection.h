
#ifndef COLLISIONDETECTION_
#define COLLISIONDETECTION_
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


const float res = 0.1;

class CollisionDetection{
public:
    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    ros::Publisher vis_pub ;
    ros::Publisher octmap_pub;
    ros::Publisher collide_F_pub ;
    ros::Publisher  vis_cude_pub ;

		CollisionDetection()
		{};

		~CollisionDetection()
		{};

 CollisionDetection(ros::NodeHandle n) ; 
    bool inCollision(sensor_msgs::LaserScan scan_in , nav_msgs::Odometry robot_pose ) ; 
    void generateBoxesFromOctomap(std::vector<CollisionObject*>& boxes, OcTree& tree) ; 
    void drawSphere(Vec3f vec ) ; 
    visualization_msgs::Marker drawCUBE(Vec3f vec , int id ) ; 

   
};

#endif

