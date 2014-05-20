/***************************************************************************
* Copyright (C) 2013 - 2014 by                                             *
* Rui Figueiredo, Khalifa University Robotics Institute KURI               *
* <rui.defigueiredo@kustar.ac.ae>                                          *
*                                                                          *
* 									   *
* This program is free software; you can redistribute it and/or modify     *
* it under the terms of the GNU General Public License as published by     *
* the Free Software Foundation; either version 2 of the License, or        *
* (at your option) any later version. 					   *
* 									   *
* This program is distributed in the hope that it will be useful, 	   *
* but WITHOUT ANY WARRANTY; without even the implied warranty of 	   *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 		   *
* GNU General Public License for more details. 				   *
* 									   *
* You should have received a copy of the GNU General Public License 	   *
* along with this program; if not, write to the 			   *
* Free Software Foundation, Inc., 					   *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA. 		   *
***************************************************************************/
#ifndef MASTER_CONTROLLER_
#define MASTER_CONTROLLER_
#include "navigation/Controller.h"

class MasterController : public Controller
{
	public:
		dynamic_reconfigure::Server<navigation::MasterControllerConfig> master_server;
  		dynamic_reconfigure::Server<navigation::MasterControllerConfig>::CallbackType master_callback_type;

		MasterController(ros::NodeHandle & n_,
                   double freq_,
                   Eigen::Matrix<double,6,1> Kp_,
                   Eigen::Matrix<double,6,1> Kd_,
                   Eigen::Matrix<double,6,1> Bd_,
                   Eigen::Matrix<double,6,6> lambda_,
                   Eigen::Matrix<double,6,1> slave_to_master_scale_,
		   Eigen::Matrix<double,6,1> slave_velocity_master_pose_scale_,
                   Eigen::Matrix<double,6,1> master_min_,
                   Eigen::Matrix<double,6,1> master_max_,
                   Eigen::Matrix<double,6,1> slave_min_,
                   Eigen::Matrix<double,6,1> slave_max_,
		   Eigen::Matrix<double,6,1> slave_velocity_min_,
		   Eigen::Matrix<double,6,1> slave_velocity_max_);

		void paramsCallback(navigation::MasterControllerConfig &config, uint32_t level);

	private:
		// MASTER MEASUREMENTS
        	void masterJointsCallback(const sensor_msgs::JointState::ConstPtr& joint_states);

		// SLAVE MEASUREMENTS
		void slaveOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
		//void MasterController::feedbackCallback(const geometry_msgs::Point::ConstPtr& force);

		void feedback();

  		void initParams();
		// subscriber 
		//ros::Subscriber force_feedback;

		Eigen::Matrix<double,6,1> slave_to_master_scale;
		Eigen::Matrix<double,6,1> slave_velocity_master_pose_scale;
};
#endif
