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


#include "nao_tactile_interface_node.h"


NaoTactileInterface::NaoTactileInterface (void) {

    // Init Subscriber
    this->subHead = this->n.subscribe("tactile_head", 10,
                &NaoTactileInterface::head_callback, this);

    this->subBumper = this->n.subscribe("bumper", 10,
                &NaoTactileInterface::bumper_callback, this);

    // Init Publisher
    this->interfacePub = this->n.advertise
                    <nao_tactile_interface::TactileInterface>("tactile_interface", 10);

    /// Init Variables
    // Init bool vars, have been already published?
    headFrontPublished = true;
    headMiddlePublished = true;
    headRearPublished = true;
    bumperLeftPublished = true;
    bumperRightPublished = true;

    // Init time vars
    for (int i=0;i<3;++i) {
        ros::Time oneHourAgo = ros::Time::now() - ros::Duration(60*60);

        headFrontPress[i] = oneHourAgo;
        headFrontRelease[i] = oneHourAgo;

        headMiddlePress[i] = oneHourAgo; 
        headMiddleRelease[i] = oneHourAgo;

        headRearPress[i] = oneHourAgo; 
        headRearRelease[i] = oneHourAgo; 

        bumperLeftPress[i] = oneHourAgo;
        bumperLeftRelease[i] = oneHourAgo; 

        bumperRightPress[i] = oneHourAgo; 
        bumperRightRelease[i] = oneHourAgo;
    }

}

NaoTactileInterface::~NaoTactileInterface (void) {
}


/** 
 *      Tactile Touch Subscriber, head buttons
 *      Detecting when a button is pressed or released
 */
void NaoTactileInterface::head_callback(const naoqi_bridge_msgs::TactileTouch::ConstPtr& msg)
{

    if (msg->button == 1) { // Head Front
        if (msg->state) {   // Pressed
            headFrontPress[2] = headFrontPress[1];
            headFrontPress[1] = headFrontPress[0];
            headFrontPress[0] = ros::Time::now();
            headFrontPublished = false;
        }
        else {              // Released
            headFrontRelease[2] = headFrontRelease[1];
            headFrontRelease[1] = headFrontRelease[0];
            headFrontRelease[0] = ros::Time::now();
        }
    }
    if (msg->button == 2) { // Head Middle
        if (msg->state) {   // Pressed
            headMiddlePress[2] = headMiddlePress[1];
            headMiddlePress[1] = headMiddlePress[0];
            headMiddlePress[0] = ros::Time::now();
            headMiddlePublished = false;
        }
        else {              // Released
            headMiddleRelease[2] = headMiddleRelease[1];
            headMiddleRelease[1] = headMiddleRelease[0];
            headMiddleRelease[0] = ros::Time::now();
        }
    }
    if (msg->button == 3) { // Head Middle
        if (msg->state) {   // Pressed
            headRearPress[2] = headRearPress[1];
            headRearPress[1] = headRearPress[0];
            headRearPress[0] = ros::Time::now();
            headRearPublished = false;
        }
        else {              // Released
            headRearRelease[2] = headRearRelease[1];
            headRearRelease[1] = headRearRelease[0]; 
            headRearRelease[0] = ros::Time::now();
        }
    }

}


/** 
 *      Bumper Subscriber
 *      Detecting when a bumper is pressed or released
 */
void NaoTactileInterface::bumper_callback(const naoqi_bridge_msgs::Bumper::ConstPtr& msg)
{

    if (msg->bumper == 0) { // Right Bumper
        if (msg->state) {   // Pressed
            bumperRightPress[2] = bumperRightPress[1];
            bumperRightPress[1] = bumperRightPress[0];
            bumperRightPress[0] = ros::Time::now();
            bumperRightPublished = false;
        }
        else {              // Released
            bumperRightRelease[2] = bumperRightRelease[1];
            bumperRightRelease[1] = bumperRightRelease[0];
            bumperRightRelease[0] = ros::Time::now();
        }
    }
    if (msg->bumper == 1) { // Left Bumper
        if (msg->state) {   // Pressed
            bumperLeftPress[2] = bumperLeftPress[1];
            bumperLeftPress[1] = bumperLeftPress[0];
            bumperLeftPress[0] = ros::Time::now();
            bumperLeftPublished = false;
        }
        else {              // Released
            bumperLeftRelease[2] = bumperLeftRelease[1];
            bumperLeftRelease[1] = bumperLeftRelease[0];
            bumperLeftRelease[0] = ros::Time::now();
        }
    }

}


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
int NaoTactileInterface::check_taps(const ros::Time timesP[3], const ros::Time timesR[3], 
                                    bool &published)
{
    ros::Time CurrentTime = ros::Time::now();
    int output=0;
    const double CLICK_TIME = 0.6;  // seconds
    const double MAX_LONG = 5.0;    // seconds, Long press no more than MAX_LONG
                                    // sometimes release is not detected, safety feature
    
    // We had a click CLICK_TIME ago and has not been published
    if (CurrentTime.toSec() > timesP[0].toSec() + CLICK_TIME and !published) {
        // We had two more clicks CLICK_TIME/2 and CLICK_TIME/2 ago -> triple Click
        if (timesP[2].toSec() + CLICK_TIME/2 > timesP[1].toSec() and
            timesP[1].toSec() + CLICK_TIME/2 > timesP[0].toSec()) {
            ROS_INFO("[nao_tactile_interface] Triple Click");
            published = true;
            output=3;
        }   
        // We had 1 more click ago CLICK_TIME/2 ago -> double click
        else if (timesP[1].toSec() + CLICK_TIME/2 > timesP[0].toSec()) {
            ROS_INFO("[nao_tactile_interface] Double Click");
            published = true;
            output=2;
        }   
        // Buttom is released in less than CLICK_TIME -> single click
        else if (timesP[0].toSec() < timesR[0].toSec() and 
                 timesR[0].toSec() - timesP[0].toSec() < CLICK_TIME ) {
            ROS_INFO("[nao_tactile_interface] Single Click");
            published = true;
            output=1;
        }
        // Still not released, during less than MAX_LONG -> long click
        else if (timesP[0].toSec() > timesR[0].toSec() and 
                 timesP[0].toSec() + MAX_LONG > CurrentTime.toSec() ) {
            ROS_INFO("[nao_tactile_interface] Long Click");
            output=4;
        }
        // Long click is released, or longer than MAX_LONG -> stop publishing
        else {
            published = true;
        }
    }

    return output;

}


int NaoTactileInterface::Main (void) 
{

    // Main Loop
    ros::Rate loop_rate(20);  // Hz 
    while (ros::ok()) {
    
        // Var used to store the message to publish
        nao_tactile_interface::TactileInterface var_pub;

        // Checking inputs (press, released) for all the buttons 
        var_pub.HeadFront = check_taps(headFrontPress, headFrontRelease, headFrontPublished);
        var_pub.HeadMiddle = check_taps(headMiddlePress, headMiddleRelease, headMiddlePublished);
        var_pub.HeadRear = check_taps(headRearPress, headRearRelease, headRearPublished);
        var_pub.BumperLeft = check_taps(bumperLeftPress, bumperLeftRelease, bumperLeftPublished);
        var_pub.BumperRight = check_taps(bumperRightPress, bumperRightRelease, bumperRightPublished);

        // Publishing, only if there is something to publish
        if ( var_pub.HeadFront   != 0 or
             var_pub.HeadMiddle  != 0 or
             var_pub.HeadRear    != 0 or
             var_pub.BumperLeft  != 0 or
             var_pub.BumperRight != 0 ) 
        {
            interfacePub.publish(var_pub);
        }

        // ROS stuff
        ros::spinOnce();
        loop_rate.sleep();
    }

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "nao_tactile_interface");

    NaoTactileInterface foo;
    return foo.Main();

}
