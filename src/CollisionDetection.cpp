#include "haptic_teleoperation/CollisionDetection.h"


CollisionDetection::CollisionDetection(ros::NodeHandle n):n_(n)
    {

        vis_pub = n_.advertise<visualization_msgs::Marker>("Sphere", 1);
        vis_cude_pub = n_.advertise<visualization_msgs::MarkerArray>("Cube", 1);
        octmap_pub = n_.advertise<octomap_msgs::Octomap>("octomap_rviz", 1);
        collide_F_pub = n_.advertise<std_msgs::Bool>("collision_flag",1) ;

    }


 bool CollisionDetection::inCollision(sensor_msgs::LaserScan scan_in , nav_msgs::Odometry robot_pose )
    {
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





        octomap::OcTree* st_tree = new octomap::OcTree(0.1);
        octomap::Pointcloud st_cld;
        for(int i = 0;i<msg.points.size();i++){
            if((msg.points[i].x*msg.points[i].x + msg.points[i].y*msg.points[i].y + msg.points[i].z + msg.points[i].z) < 2 )
            {
                octomap::point3d endpoint((float) msg.points[i].x,(float) msg.points[i].y,(float) msg.points[i].z);
                st_cld.push_back(endpoint);
            }
        }


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
        Transform3f tf0, tf1;
        tf0.setIdentity();
        tf0.setTranslation(Vec3f(robot_pose->pose.pose.position.x,robot_pose->pose.pose.position.y,robot_pose->pose.pose.position.z));
        tf1.setIdentity();
        CollisionObject co0(Shpere0, tf0);
        AABB a = co0.getAABB() ;
        Vec3f vec =  a.center() ;
        drawSphere( vec ) ;


        std::vector<CollisionObject*> boxes;
        generateBoxesFromOctomap(boxes, *st_tree2);


        visualization_msgs::MarkerArray marker_array ;
        for(size_t i = 0; i <= boxes.size(); ++i)
        {
            CollisionObject* box =  boxes[i];

            static const int num_max_contacts = std::numeric_limits<int>::max();
            static const bool enable_contact = true ;

            AABB b = box->getAABB() ;
            Vec3f vec2 =  b.center();

            visualization_msgs::Marker marker = drawCUBE(vec2, i) ;
            marker_array.markers.push_back(marker);
            fcl::CollisionResult result;
            fcl::CollisionRequest request(num_max_contacts, enable_contact);
            fcl::collide(&co0, box, request, result);

	    vis_cude_pub.publish(marker_array);
            for(size_t i = 0; i < boxes.size(); ++i)
            delete boxes[i];
            
            if (result.isCollision() == true )
            {
                std::cout << "EXIT " << std::endl ;
                return true ; 

            }
            else
            {
		return false; 
            }
        }
        


    }



    void CollisionDetection::generateBoxesFromOctomap(std::vector<CollisionObject*>& boxes, OcTree& tree)
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


    void CollisionDetection::drawSphere(Vec3f vec )
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        // if(shape == "Sphere")
        //   marker.type = visualization_msgs::Marker::SPHERE;
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
        marker.scale.x = 1;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration();
        //only if using a MESH_RESOURCE marker type:
        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        vis_pub.publish( marker );
    }

    visualization_msgs::Marker CollisionDetection::drawCUBE(Vec3f vec , int id )
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
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
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.lifetime = ros::Duration();
        //only if using a MESH_RESOURCE marker type:
        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        return marker ;
    }
