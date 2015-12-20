/**
 *      ROS subscriber for the Nao robot to detect which kind of
 *      tactile input is received: Bumper, Head, Press, Release.
 *
 *      GNU General Public License v3.0 
 *      Copyright (c) 2015 Felip Marti Carrillo  
 */


#include "ros/ros.h"

#include "naoqi_bridge_msgs/TactileTouch.h"
#include "naoqi_bridge_msgs/Bumper.h"


void headCallback(const naoqi_bridge_msgs::TactileTouch::ConstPtr& msg)
{

    if (msg->button == 1) {
        if (msg->state)
            ROS_INFO("Front Button has been Pressed");
        else
            ROS_INFO("Front Button has been Released");
    }
    if (msg->button == 2) {
        if (msg->state)
            ROS_INFO("Middle Button has been Pressed");
        else
            ROS_INFO("Middle Button has been Released");
    }
    if (msg->button == 3) {
        if (msg->state)
            ROS_INFO("Rear Button has been Pressed");
        else
            ROS_INFO("Rear Button has been Released");
    }

}


void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr& msg)
{

    if (msg->bumper == 0) {
        if (msg->state)
            ROS_INFO("Right Bumper has been Pressed");
        else
            ROS_INFO("Right Bumper has been Released");
    }
    if (msg->bumper == 1) {
        if (msg->state)
            ROS_INFO("Left Bumper has been Pressed");
        else
            ROS_INFO("Left Bumper has been Released");
    }

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "nao_tactile_detection");
    ros::NodeHandle n;
    ros::Subscriber subhead = n.subscribe("tactile_head", 1000, headCallback);
    ros::Subscriber subbumper = n.subscribe("bumper", 1000, bumperCallback);

    ROS_INFO("Please, touch Nao Buttons and Bumpers");

    ros::spin();
    return 0;
}
