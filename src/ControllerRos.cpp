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


#include "haptic_teleoperation/MasterController.h"
#include "haptic_teleoperation/SlaveController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");

    // parameters
    double freq;
    n_priv.param<double>("frequency", freq, 10.0);

    double kp_x;
    double kp_y;
    double kp_z;
    double kp_roll;
    double kp_pitch;
    double kp_yaw;

    n_priv.param<double>("kp_x",     kp_x, 1.0);
    n_priv.param<double>("kp_y",     kp_y, 1.0);
    n_priv.param<double>("kp_z",     kp_z, 1.0);
    n_priv.param<double>("kp_roll",  kp_roll, 1.0);
    n_priv.param<double>("kp_pitch", kp_pitch, 1.0);
    n_priv.param<double>("kp_yaw",   kp_yaw, 1.0);

    Eigen::Matrix<double,6,1> Kp;
    Kp << 	kp_x,
            kp_y,
            kp_z,
            kp_roll,
            kp_pitch,
            kp_yaw;

    double kd_x;
    double kd_y;
    double kd_z;
    double kd_roll;
    double kd_pitch;
    double kd_yaw;
    n_priv.param<double>("kd_x",     kd_x, 1.0);
    n_priv.param<double>("kd_y",     kd_y, 1.0);
    n_priv.param<double>("kd_z",     kd_z, 1.0);
    n_priv.param<double>("kd_roll",  kd_roll, 1.0);
    n_priv.param<double>("kd_pitch", kd_pitch, 1.0);
    n_priv.param<double>("kd_yaw",   kd_yaw, 1.0);

    Eigen::Matrix<double,6,1> Kd;
    Kd << 	kd_x,
            kd_y,
            kd_z,
            kd_roll,
            kd_pitch,
            kd_yaw;

    double bd_x;
    double bd_y;
    double bd_z;
    double bd_roll;
    double bd_pitch;
    double bd_yaw;
    n_priv.param<double>("bd_x",     bd_x, 1.0);
    n_priv.param<double>("bd_y",     bd_y, 1.0);
    n_priv.param<double>("bd_z",     bd_z, 1.0);
    n_priv.param<double>("bd_roll",  bd_roll, 1.0);
    n_priv.param<double>("bd_pitch", bd_pitch, 1.0);
    n_priv.param<double>("bd_yaw",   bd_yaw, 1.0);

    Eigen::Matrix<double,6,1> Bd;
    Bd << 	bd_x,
            bd_y,
            bd_z,
            bd_roll,
            bd_pitch,
            bd_yaw;


    double fp_x;
    double fp_y;
    double fp_z;
    double fp_roll;
    double fp_pitch;
    double fp_yaw;
    n_priv.param<double>("fp_x",     fp_x, 1.0);
    n_priv.param<double>("fp_y",     fp_y, 1.0);
    n_priv.param<double>("fp_z",     fp_z, 1.0);
    n_priv.param<double>("fp_roll",  fp_roll, 1.0);
    n_priv.param<double>("fp_pitch", fp_pitch, 1.0);
    n_priv.param<double>("fp_yaw",   fp_yaw, 1.0);

    Eigen::Matrix<double,6,1> Fp;
    Fp << 	fp_x,
            fp_y,
            fp_z,
            fp_roll,
            fp_pitch,
            fp_yaw;


    double lambda_x;
    double lambda_y;
    double lambda_z;
    double lambda_roll;
    double lambda_pitch;
    double lambda_yaw;
    n_priv.param<double>("lambda_x",     lambda_x, 1.0);
    n_priv.param<double>("lambda_y",     lambda_y, 1.0);
    n_priv.param<double>("lambda_z",     lambda_z, 1.0);
    n_priv.param<double>("lambda_roll",  lambda_roll, 0.0);
    n_priv.param<double>("lambda_pitch", lambda_pitch, 0.0);
    n_priv.param<double>("lambda_yaw",   lambda_yaw, 0.0);

    Eigen::Matrix<double,6,6> lambda;
    lambda << lambda_x, 0, 0, 0 ,0, 0,
            0, lambda_y, 0, 0, 0, 0,
            0, 0, lambda_z, 0, 0, 0,
            0, 0, 0, lambda_roll, 0, 0,
            0, 0, 0, 0, lambda_pitch, 0,
            0, 0, 0, 0, 0, lambda_yaw;

    double slave_min_x;
    double slave_min_y;
    double slave_min_z;
    double slave_min_roll;
    double slave_min_pitch;
    double slave_min_yaw;
    n_priv.param<double>("slave_min_x",     slave_min_x, 1.0);
    n_priv.param<double>("slave_min_y",     slave_min_y, 1.0);
    n_priv.param<double>("slave_min_z",     slave_min_z, 1.0);
    n_priv.param<double>("slave_min_roll",  slave_min_roll, 1.0);
    n_priv.param<double>("slave_min_pitch", slave_min_pitch, 1.0);
    n_priv.param<double>("slave_min_yaw",   slave_min_yaw, 1.0);
    Eigen::Matrix<double,6,1> slave_min;
    slave_min << slave_min_x,
            slave_min_y,
            slave_min_z,
            slave_min_roll,
            slave_min_pitch,
            slave_min_yaw;

    double slave_max_x;
    double slave_max_y;
    double slave_max_z;
    double slave_max_roll;
    double slave_max_pitch;
    double slave_max_yaw;
    n_priv.param<double>("slave_max_x",     slave_max_x, 1.0);
    n_priv.param<double>("slave_max_y",     slave_max_y, 1.0);
    n_priv.param<double>("slave_max_z",     slave_max_z, 1.0);
    n_priv.param<double>("slave_max_roll",  slave_max_roll,  1.0);
    n_priv.param<double>("slave_max_pitch", slave_max_pitch, 1.0);
    n_priv.param<double>("slave_max_yaw",   slave_max_yaw,   1.0);
    Eigen::Matrix<double,6,1> slave_max;
    slave_max << slave_max_x,
            slave_max_y,
            slave_max_z,
            slave_max_roll,
            slave_max_pitch,
            slave_max_yaw;
    Eigen::Matrix<double,6,1> slave_size=slave_max-slave_min;

    double slave_velocity_min_x;
    double slave_velocity_min_y;
    double slave_velocity_min_z;
    double slave_velocity_min_roll;
    double slave_velocity_min_pitch;
    double slave_velocity_min_yaw;
    n_priv.param<double>("slave_velocity_min_x",     slave_velocity_min_x, 1.0);
    n_priv.param<double>("slave_velocity_min_y",     slave_velocity_min_y, 1.0);
    n_priv.param<double>("slave_velocity_min_z",     slave_velocity_min_z, 1.0);
    n_priv.param<double>("slave_velocity_min_roll",  slave_velocity_min_roll, 1.0);
    n_priv.param<double>("slave_velocity_min_pitch", slave_velocity_min_pitch, 1.0);
    n_priv.param<double>("slave_velocity_min_yaw",   slave_velocity_min_yaw, 1.0);
    Eigen::Matrix<double,6,1> slave_velocity_min;
    slave_velocity_min << slave_velocity_min_x,
            slave_velocity_min_y,
            slave_velocity_min_z,
            slave_velocity_min_roll,
            slave_velocity_min_pitch,
            slave_velocity_min_yaw;

    double slave_velocity_max_x;
    double slave_velocity_max_y;
    double slave_velocity_max_z;
    double slave_velocity_max_roll;
    double slave_velocity_max_pitch;
    double slave_velocity_max_yaw;

    n_priv.param<double>("slave_velocity_max_x",     slave_velocity_max_x, 1.0);
    n_priv.param<double>("slave_velocity_max_y",     slave_velocity_max_y, 1.0);
    n_priv.param<double>("slave_velocity_max_z",     slave_velocity_max_z, 1.0);
    n_priv.param<double>("slave_velocity_max_roll",  slave_velocity_max_roll, 1.0);
    n_priv.param<double>("slave_velocity_max_pitch", slave_velocity_max_pitch, 1.0);
    n_priv.param<double>("slave_velocity_max_yaw",   slave_velocity_max_yaw, 1.0);
    Eigen::Matrix<double,6,1> slave_velocity_max;
    slave_velocity_max << slave_velocity_max_x,
            slave_velocity_max_y,
            slave_velocity_max_z,
            slave_velocity_max_roll,
            slave_velocity_max_pitch,
            slave_velocity_max_yaw;



    Eigen::Matrix<double,6,1> slave_velocity_size=slave_velocity_max-slave_velocity_min;

    double master_min_x;
    double master_min_y;
    double master_min_z;
    double master_min_roll;
    double master_min_pitch;
    double master_min_yaw;
    n_priv.param<double>("master_min_x",     master_min_x,     1.0);
    n_priv.param<double>("master_min_y",     master_min_y,     1.0);
    n_priv.param<double>("master_min_z",     master_min_z,     1.0);
    n_priv.param<double>("master_min_roll",  master_min_roll,  1.0);
    n_priv.param<double>("master_min_pitch", master_min_pitch, 1.0);
    n_priv.param<double>("master_min_yaw",   master_min_yaw,   1.0);
    Eigen::Matrix<double,6,1> master_min;
    master_min << master_min_x,
            master_min_y,
            master_min_z,
            master_min_roll,
            master_min_pitch,
            master_min_yaw;

    double master_max_x;
    double master_max_y;
    double master_max_z;
    double master_max_roll;
    double master_max_pitch;
    double master_max_yaw;
    n_priv.param<double>("master_max_x",     master_max_x,     1.0);
    n_priv.param<double>("master_max_y",     master_max_y,     1.0);
    n_priv.param<double>("master_max_z",     master_max_z,     1.0);
    n_priv.param<double>("master_max_roll",  master_max_roll,  1.0);
    n_priv.param<double>("master_max_pitch", master_max_pitch, 1.0);
    n_priv.param<double>("master_max_yaw",   master_max_yaw,   1.0);
    Eigen::Matrix<double,6,1> master_max;
    master_max << master_max_x,
            master_max_y,
            master_max_z,
            master_max_roll,
            master_max_pitch,
            master_max_yaw;

    Eigen::Matrix<double,6,1> master_size=master_max-master_min;
    std::cout << "SLAVE VELOCITY SIZE:" << slave_velocity_size << std::endl;


    bool is_master;
    n_priv.param<bool>("is_master", is_master, true);

    //Controller controller;
    if(is_master)
    {
        Eigen::Matrix<double,6,1> slave_to_master_scale;
        slave_to_master_scale << fabs(master_size(0,0)/slave_size(0,0)),
                fabs(master_size(1,0)/slave_size(1,0)),
                fabs(master_size(2,0)/slave_size(2,0)),
                fabs(master_size(3,0)/slave_size(3,0)),
                fabs(master_size(4,0)/slave_size(4,0)),
                fabs(master_size(5,0)/slave_size(5,0));
        Eigen::Matrix<double,6,1> slave_velocity_master_pose_scale;
        slave_velocity_master_pose_scale << fabs(master_size(0,0)/slave_velocity_size(0,0)),
                fabs(master_size(1,0)/slave_velocity_size(1,0)),
                fabs(master_size(2,0)/slave_velocity_size(2,0)),
                fabs(master_size(3,0)/slave_velocity_size(3,0)),
                fabs(master_size(4,0)/slave_velocity_size(4,0)),
                fabs(master_size(5,0)/slave_velocity_size(5,0));

        MasterController controller(n, freq, Kp, Kd, Bd, Fp, lambda, slave_to_master_scale, slave_velocity_master_pose_scale, master_min, master_max, slave_min, slave_max, slave_velocity_min, slave_velocity_max);
        ros::Rate loop_rate(freq);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    else
    {
        Eigen::Matrix<double,6,1> master_to_slave_scale;
        master_to_slave_scale << fabs(slave_size(0,0)/master_size(0,0)),
                fabs(slave_size(1,0)/master_size(1,0)),
                fabs(slave_size(2,0)/master_size(2,0)),
                fabs(slave_size(3,0)/master_size(3,0)),
                fabs(slave_size(4,0)/master_size(4,0)),
                fabs(slave_size(5,0)/master_size(5,0));
        Eigen::Matrix<double,6,1> master_pose_slave_velocity_scale;
        master_pose_slave_velocity_scale << fabs(slave_velocity_size(0,0)/master_size(0,0)),
                fabs(slave_velocity_size(1,0)/master_size(1,0)),
                fabs(slave_velocity_size(2,0)/master_size(2,0)),
                fabs(slave_velocity_size(3,0)/master_size(3,0)),
                fabs(slave_velocity_size(4,0)/master_size(4,0)),
                fabs(slave_velocity_size(5,0)/master_size(5,0));
        std::cout << master_size(0,0) << " " <<  slave_velocity_size(0,0) << std::endl;
        std::cout << "before:" <<master_pose_slave_velocity_scale << std::endl;
        std::cout << "velocities min " << slave_min.transpose() << std::endl ;
        std::cout << "velocities max " << slave_max.transpose() << std::endl ;

        SlaveController controller(n, freq, Kp, Kd, Bd,Fp, lambda, master_to_slave_scale, master_pose_slave_velocity_scale, master_min, master_max, slave_min, slave_max, slave_velocity_min, slave_velocity_max);
        ros::Rate loop_rate(freq);

        while (ros::ok())
        {

            ros::spinOnce();

            loop_rate.sleep();
        }
    }

    return 0;
}
