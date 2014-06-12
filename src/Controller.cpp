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

#include "haptic_teleoperation/Controller.h"


Controller::Controller(ros::NodeHandle & n_,
                       double freq_,
                       Eigen::Matrix<double,6,1> Kp_,
                       Eigen::Matrix<double,6,1> Kd_,
                       Eigen::Matrix<double,6,1> Bd_,
                       Eigen::Matrix<double,6,1> Fp_,
                       Eigen::Matrix<double,6,6> lambda_,
                       Eigen::Matrix<double,6,1> master_min_,
                       Eigen::Matrix<double,6,1> master_max_,
                       Eigen::Matrix<double,6,1> slave_min_,
                       Eigen::Matrix<double,6,1> slave_max_,
                       Eigen::Matrix<double,6,1> slave_velocity_min_,
                       Eigen::Matrix<double,6,1> slave_velocity_max_) :
    n(n_),
    n_priv("~"),
    freq(freq_),
    Kp(Kp_),
    Kd(Kd_),
    Bd(Bd_),
    Fp(Fp_),
    lambda(lambda_),
    master_min(master_min_),
    master_max(master_max_),
    slave_min(slave_min_),
    slave_max(slave_max_),
    slave_velocity_min(slave_velocity_min_),
    slave_velocity_max(slave_velocity_max_),
    master_new_readings(false),
    slave_new_readings(false),
    control_event(false),
    linear_button_pressed(false),
    angular_button_pressed(false),
    yaw_master_joint_previous(0.0),
    yaw_slave_previous(0.0),
    init_slave_readings(false)
{
    previous_time=ros::Time::now();
    button_sub = n_.subscribe<phantom_omni::PhantomButtonEvent>("/omni1_button", 1, &Controller::buttonCallback, this);
}


// HAPTIC BUTTON
void Controller::buttonCallback(const phantom_omni::PhantomButtonEvent::ConstPtr& button)
{
    if(button->grey_button==1)
        linear_button_pressed=true;
    else
        linear_button_pressed=false;


    if(button->white_button==1)
        angular_button_pressed=true;
    else
        angular_button_pressed=false;

    if(linear_button_pressed || angular_button_pressed)
        control_event=true;
    else
        control_event=false;
}

