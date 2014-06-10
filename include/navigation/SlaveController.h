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
#ifndef SLAVE_CONTROLLER_
#define SLAVE_CONTROLLER_
#include "navigation/Controller.h"
#include "ardrone_autonomy/Navdata.h"

class SlaveController : public Controller
{
	public:
  		dynamic_reconfigure::Server<navigation::SlaveControllerConfig> slave_server;
  		dynamic_reconfigure::Server<navigation::SlaveControllerConfig>::CallbackType slave_callback_type;

		SlaveController(ros::NodeHandle & n_,
                  double freq_,
                  Eigen::Matrix<double,6,1> Kp_,
                  Eigen::Matrix<double,6,1> Kd_,
                  Eigen::Matrix<double,6,1> Bd_,
                   Eigen::Matrix<double,6,1> Fp_,
                  Eigen::Matrix<double,6,6> lambda_,
                  Eigen::Matrix<double,6,1> master_to_slave_scale_,
		  Eigen::Matrix<double,6,1> master_pose_slave_velocity_scale_,
                  Eigen::Matrix<double,6,1> master_min_,
                  Eigen::Matrix<double,6,1> master_max_,
                  Eigen::Matrix<double,6,1> slave_min_,
                  Eigen::Matrix<double,6,1> slave_max_,
		  Eigen::Matrix<double,6,1> slave_velocity_min_,
		  Eigen::Matrix<double,6,1> slave_velocity_max_);

		void paramsCallback(navigation::SlaveControllerConfig &config, uint32_t level);

	private:
		// MASTER MEASUREMENTS
		void masterJointsCallback(const sensor_msgs::JointState::ConstPtr& joint_states);

		// SLAVE MEASUREMENTS
		void slaveOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    		void get_navdata(const ardrone_autonomy::Navdata::ConstPtr& msg);

		void feedback();

  		void initParams();


		Eigen::Matrix<double,6,1> master_to_slave_scale;
		Eigen::Matrix<double,6,1> master_pose_slave_velocity_scale;

};
#endif
