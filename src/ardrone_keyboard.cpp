/*
* Copyright (C) 2013 - 2014 by                                             *
* reem ashour, Khalifa University Robotics Institute KURI               *
* <reem.ashour@kustar.ac.ae>                                          *
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
 */

#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include<stdio.h>
#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO


int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotsTeleop");
    ros::NodeHandle nh_;
    std_srvs::Empty empty;
    std_msgs::Empty msg ;

    ros::Publisher take_off_pub = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1 );
    ros::Publisher land_pub = nh_.advertise<std_msgs::Empty>("/ardrone/land",1 );
    ros::Publisher eme_pub = nh_.advertise<std_msgs::Empty>("ardrone/reset",1);
    ros::ServiceClient flat_trim_client = nh_.serviceClient<std_srvs::Empty>("/ardrone/flattrim");
    while (ros::ok())
    {
        int c = getch();   // call your non-blocking input function
        if (c == 't' || c=='T') // take off
        {
            std::cout << "takeoff" << std::endl ;
            double time_start=(double)ros::Time::now().toSec();
            // flat trim before taking off
            flat_trim_client.call(empty);
            flat_trim_client.call(empty);
            while ((double)ros::Time::now().toSec()< time_start+5.0) /* Send command for five seconds*/
            {
                std::cout << "Inside while loop" << std::endl ;
                take_off_pub.publish(msg) ;
                ros::spinOnce();
            }//time loop
            ROS_INFO("ARdrone launched");

        }

        else if (c == 'l' || c=='L') // landing
        {
            std::cout << "land" << std::endl ;

            double time_start=(double)ros::Time::now().toSec();
            while ((double)ros::Time::now().toSec()< time_start+5.0)
            {
                land_pub.publish(msg);
                ros::spinOnce();

            }
            ROS_INFO("ARdrone landed");
            exit(0);
        }
        else if (c == 'r' || c =='R') // reset
        {
            std::cout << "emergency" << std::endl ;

            double time_start=(double)ros::Time::now().toSec();
            while ((double)ros::Time::now().toSec()< time_start+1.0)
            {
                eme_pub.publish(msg);
                ros::spinOnce();
            }
            ROS_INFO("ARdrone switch");
        }
    }

}


// http://answers.ros.org/question/63491/keyboard-key-pressed/
