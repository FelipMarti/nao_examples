/**
 *      ROS service to blink Nao brain leds with 
 *      different effects.
 *
 *      GNU General Public License v3.0 
 *      Copyright (c) 2016 Felip Marti Carrillo  
 */

#include "nao_leds_effects_node.h"


NaoLedsEffects::NaoLedsEffects (void) 
{

    // Init Pubisher
    this->ledPub = this->n.advertise<naoqi_bridge_msgs::FadeRGB>("leds", 1000);

    // Init Service
    this->ledsService = this->n.advertiseService("leds_effects", &NaoLedsEffects::change_effect, this);


    // Init Led Effects
    timer100ms = 0;
    typeLedEffect = 0;

}

NaoLedsEffects::~NaoLedsEffects (void) 
{
}


void NaoLedsEffects::brain_led_on(std::string s, double fadeDuration) 
{
    naoqi_bridge_msgs::FadeRGB msg;
    msg.led_name=s;
    msg.color.r=1;
    msg.color.g=1;
    msg.color.b=1;
    msg.fade_duration= ros::Duration(fadeDuration);
    ledPub.publish(msg);
    ROS_INFO("[nao_leds_effects] %s ON",s.c_str());
}


void NaoLedsEffects::brain_leds_off(double fadeDuration) 
{
    naoqi_bridge_msgs::FadeRGB msg;
    msg.led_name="BrainLeds";
    msg.color.r=0;
    msg.color.g=0;
    msg.color.b=0;
    msg.fade_duration= ros::Duration(fadeDuration);
    ledPub.publish(msg);
    ROS_INFO("[nao_leds_effects] Brain Leds OFF");
}


void NaoLedsEffects::blink_fast_brain_leds(void) 
{
    // Trying to divide 2s by 6. 3 times on, 3 off
    if (timer100ms == 0 or timer100ms == 7 or timer100ms == 14 ) { // ON
        brain_led_on("BrainLeds");
    }
    else if (timer100ms == 3 or timer100ms == 10 or timer100ms == 17) { // OFF 
        brain_leds_off();
    }
}


void NaoLedsEffects::blink_brain_leds(void) 
{
    // In 2s, 2 times on 2 times off
    if (timer100ms == 0 or timer100ms == 10) {      // ON
        brain_led_on("BrainLeds");
    }
    else if (timer100ms == 5 or timer100ms == 15) { // OFF
        brain_leds_off();
    }
}


void NaoLedsEffects::left_right_brain_leds(void) 
{
    // In 2s, left-right leds on off with fade 
    if (timer100ms == 0) {      
        brain_leds_off(0.4);
    }
    else if (timer100ms == 5) {      
        brain_led_on("BrainLedsLeft",0.4);
    }
    else if (timer100ms == 10) { 
        brain_leds_off(0.4);
    }
    else if (timer100ms == 15) { 
        brain_led_on("BrainLedsRight",0.4);
    }
}


void NaoLedsEffects::paused_brain_leds(void) 
{
    // Paused leds configuration
    if (timer100ms == 0) {      
        brain_leds_off(0);
    }
    else if (timer100ms == 5) {      
        brain_led_on("BrainLedsFront");
        brain_led_on("BrainLedsBack");
    }
    else if (timer100ms == 15) { 
        brain_led_on("BrainLedsMiddle");
    }
}

// Service Callback, receiving new leds effect 
bool NaoLedsEffects::change_effect(nao_leds_effects::LedEffect::Request &req,
                                   nao_leds_effects::LedEffect::Response &res)
{
    typeLedEffect = req.effect;
    return true;
}


int NaoLedsEffects::Main (void) 
{

    ros::Rate loop_rate(10);
    // Loop speed 10Hz -> 0.1s

    while (ros::ok()) {
    
        switch (typeLedEffect) {
        case BLINKBRAINLEDS:
            blink_brain_leds();            
            break;
        case BLINKFASTBRAINLEDS:
            blink_fast_brain_leds();            
            break;
        case LEFTRIGHTBRAINLEDS:
            left_right_brain_leds();            
            break;
        case PAUSEDBRAINLEDS:
            paused_brain_leds();
            break;
        case NORMALBEHAVIOUR:
            brain_led_on("BrainLeds");
            typeLedEffect=0;
            break;
        default:
            break;
        }

        timer100ms = (timer100ms+1)%20;    

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "nao_leds_effects");

    NaoLedsEffects foo;
    return foo.Main();

}

