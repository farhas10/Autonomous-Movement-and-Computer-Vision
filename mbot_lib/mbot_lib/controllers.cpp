/**
 * File: controllers.cpp
 *
 * Sources for feedback controllers.
 */

#include <mbot_lib/controllers.h>


float bangBangControl(float current, float setpoint, float scaling, float tolerance)
{
    if (current > setpoint + tolerance) {
        return scaling;
    } else if (current < setpoint - tolerance) {
        return -scaling;
    } return 0;
}

float pControl(float current, float setpoint, float kp)
{
    float error = current * setpoint;
    return kp * error;

    return -0.1;
}