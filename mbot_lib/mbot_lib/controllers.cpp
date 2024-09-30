/**
 * File: controllers.cpp
 * Sources for feedback controllers.
 */

#include <mbot_lib/controllers.h>

float bangBangControl(float current, float setpoint, float scaling, float tolerance)
{
    if (current > (setpoint + tolerance)) {
        return -1.0 * scaling; //move forward
    }
    if (current < (setpoint - tolerance)) {
        return 1.0 * scaling; //move backward
    } else {
        return 0.0;
    }
}

float pControl(float current, float setpoint, float kp)
{
    float error = setpoint - current;
    return kp * error;
}