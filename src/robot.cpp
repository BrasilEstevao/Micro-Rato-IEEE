#include <Arduino.h>
#include "robot.h"


void robot_t::setRobotVW(float Vnom, float Wnom)
{
 
 
}

void robot_t::setState(byte new_state){
    tes = millis();
    tis = 0;
    state = new_state;
}