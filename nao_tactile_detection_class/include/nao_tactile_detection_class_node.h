/**
 *      ROS subscriber for the Nao robot to detect which kind of
 *      tactile input is received: Bumper, Head, Press, Release.
 *      But, this time with class, like a sir.
 *
 *      GNU General Public License v3.0 
 *      Copyright (c) 2015 Felip Marti Carrillo  
 */


#ifndef _NAO_TACTILE_DETECTION_CLASS_HPP
#define _NAO_TACTILE_DETECTION_CLASS_HPP

#include "ros/ros.h"

#include "naoqi_bridge_msgs/HeadTouch.h"
#include "naoqi_bridge_msgs/Bumper.h"


class NaoTactileDetectionClass {


private:


    ros::NodeHandle n;

    // [subscriber attributes]
    ros::Subscriber subHead;
    ros::Subscriber subBumper; 
    void head_callback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& msg);
    void bumper_callback(const naoqi_bridge_msgs::Bumper::ConstPtr& msg);


public:

    /**
     *  NaoTactileDetectionClass Constructor 
     */
    NaoTactileDetectionClass (void);

    /**
     *  NaoTactileDetectionClass Destructor 
     */
    ~NaoTactileDetectionClass (void);

    int Main ();


};

#endif
