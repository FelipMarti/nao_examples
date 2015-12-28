#include "ros/ros.h"
#include "naoqi_bridge_msgs/FadeRGB.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "nao_blink_brain");
    ros::NodeHandle n;
    ros::Publisher led_pub = n.advertise<naoqi_bridge_msgs::FadeRGB>("leds", 1);
    ros::Rate loop_rate(1);

    int state = 0;
    while (ros::ok()) {
    
        naoqi_bridge_msgs::FadeRGB msg;
        if (state) {    // ON
            ROS_INFO("[nao_blink_brain] ON");

            msg.led_name="BrainLeds";
            msg.color.r=1;
            msg.color.g=1;
            msg.color.b=1;
            msg.fade_duration= ros::Duration(0.0);

            state = 0;
        }
        else {          // OFF
            ROS_INFO("[nao_blink_brain] OFF");

            msg.led_name="BrainLeds";
            msg.color.r=0;
            msg.color.g=0;
            msg.color.b=0;
            msg.fade_duration= ros::Duration(0.0);
            state = 1;
        }


        led_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}

