/**
 *      ROS service to blink Nao brain leds with 
 *      different effects.
 *
 *      GNU General Public License v3.0 
 *      Copyright (c) 2016 Felip Marti Carrillo  
 */


#ifndef _NAO_LEDS_EFFECTS_NODE_HPP
#define _NAO_LEDS_EFFECTS_NODE_HPP_

#include "ros/ros.h"
#include "string.h"
#include "naoqi_bridge_msgs/FadeRGB.h"
#include "nao_leds_effects/LedEffect.h"

#define NORMALBEHAVIOUR 1 
#define BLINKBRAINLEDS 2 
#define BLINKFASTBRAINLEDS 3 
#define PAUSEDBRAINLEDS 4 
#define LEFTRIGHTBRAINLEDS 5 

class NaoLedsEffects {

private:

    ros::NodeHandle n;

    // [Publisher attributes]
    ros::Publisher ledPub;

    // [Service attributes}
    ros::ServiceServer ledsService;
    bool change_effect(nao_leds_effects::LedEffect::Request  &req,
                       nao_leds_effects::LedEffect::Response  &res);

    // Led Effects Variables
    int timer100ms;
    int typeLedEffect;

    // Led Effects Functions
    /**
     *  brain_led_on
     *  Given the name of a led or group of leds, 
     *  it switch on those leds with the indicated fade duration
     */
    void brain_led_on(std::string s, double fadeDuration=0.0); 
    /**
     *  brain_leds_off
     *  It switch off all brain leds with the indicated fade duration
     */
    void brain_leds_off(double fadeDuration=0.0); 
    /**
     *  blink_brain_leds
     *  Blinks brain leds with a 0.5s rate
     */
    void blink_brain_leds(); 
    /**
     *  blink_fast_brain_leds
     *  Blinks brain leds with a 0.3s rate
     */
    void blink_fast_brain_leds(); 
    /**
     *  left_right_brain_leds
     *  Blinks brain leds left right with a 2s rate and 0.4s fade
     */
    void left_right_brain_leds(); 
    /**
     *  paused_brain_leds
     *  Blinks brain leds following the pause pattern
     */
    void paused_brain_leds(); 

public:

    /**
     *  NaoLedsEffects Constructor
     */
    NaoLedsEffects (void);

    /**
     *  NaoLedsEffects Destructor
     */
    ~NaoLedsEffects (void);

    int Main ();

};

#endif
