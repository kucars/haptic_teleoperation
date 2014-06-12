/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
        if (c == 't')
        {
            std::cout << "takeoff" << std::endl ;
            double time_start=(double)ros::Time::now().toSec();
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

        else if (c == 'l')
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
        else if (c == 'e')
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
