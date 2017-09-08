/**
 *      ROS subscriber for the Nao robot to detect which kind of
 *      tactile input is received: Bumper, Head, Press, Release.
 *      But, this time with class, like a sir.
 *
 *      GNU General Public License v3.0 
 *      Copyright (c) 2015 Felip Marti Carrillo  
 */


#include "nao_tactile_detection_class_node.h"


NaoTactileDetectionClass::NaoTactileDetectionClass (void) {

    // Init Subscriber
    this->subHead = this->n.subscribe("tactile_head", 1000,
                &NaoTactileDetectionClass::head_callback, this);

    this->subBumper = this->n.subscribe("bumper", 1000,
                &NaoTactileDetectionClass::bumper_callback, this);

}

NaoTactileDetectionClass::~NaoTactileDetectionClass (void) {
}



void NaoTactileDetectionClass::head_callback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& msg)
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


void NaoTactileDetectionClass::bumper_callback(const naoqi_bridge_msgs::Bumper::ConstPtr& msg)
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



int NaoTactileDetectionClass::Main () 
{
    // Wait for callbacks
    ros::spin();
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "nao_tactile_detection_class");

    NaoTactileDetectionClass foo;
    return foo.Main();

}
