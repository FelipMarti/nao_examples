/**
 *      Nao tactile interface to detect different kind of inputs
 *      One tap, Double tap, Triple tap, or Long tap.
 *      Subscribed to tactile_head and bumper topics
 *      Publishing results at tactile_interface topic
 *          0 -> Not pressed
 *          1 -> Single Click
 *          2 -> Double Click
 *          3 -> Triple Click
 *          4 -> Long Click
 *
 *      GNU General Public License v3.0 
 *      Copyright (c) 2015 Felip Marti Carrillo  
 */


#ifndef _NAO_TACTILE_INTERFACE_HPP
#define _NAO_TACTILE_INTERFACE_HPP

#include "ros/ros.h"
#include "time.h"
#include "naoqi_bridge_msgs/HeadTouch.h"
#include "naoqi_bridge_msgs/Bumper.h"
#include "nao_tactile_interface/TactileInterface.h"


class NaoTactileInterface {


private:

    ros::NodeHandle n;

    // [subscriber attributes]
    ros::Subscriber subHead;
    ros::Subscriber subBumper; 
    void head_callback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& msg);
    void bumper_callback(const naoqi_bridge_msgs::Bumper::ConstPtr& msg);

    // [pubisher attributes]
    ros::Publisher interfacePub;

    // Functions
    /** 
     *      Function to check the number of Taps (Nao tactile inputs)
     *      Inputs: Last 3 times when pressed
     *              Last 3 times when released
     *              Bool to indicate if a tap has been prepared to publish 
     *      Output: id with tap type
     *              0 -> Not pressed
     *              1 -> Single Click
     *              2 -> Double Click
     *              3 -> Triple Click
     *              4 -> Long Click
     */
    int check_taps(const ros::Time timesPress[3], const ros::Time timesRelease[3], 
                   bool &published);

    // Variables
    bool headFrontPublished;
    bool headMiddlePublished;
    bool headRearPublished;
    bool bumperLeftPublished;
    bool bumperRightPublished;

    ros::Time headFrontPress[3];
    ros::Time headFrontRelease[3];

    ros::Time headMiddlePress[3];
    ros::Time headMiddleRelease[3];

    ros::Time headRearPress[3];
    ros::Time headRearRelease[3];

    ros::Time bumperLeftPress[3];
    ros::Time bumperLeftRelease[3];

    ros::Time bumperRightPress[3];
    ros::Time bumperRightRelease[3];


public:

    /**
     *  NaoTactileInterface Constructor 
     */
    NaoTactileInterface (void);

    /**
     *  NaoTactileInterface Destructor 
     */
    ~NaoTactileInterface (void);

    int Main ();


};

#endif
