/**
 * File: drive_star.cpp
 *
 * Code to drive in a five-pointed star shape without turning. 
 */

#include <iostream>
#include <cmath>

#include <signal.h>

#include <mbot_bridge/robot.h>

#include <mbot_lib/utils.h>


bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


int main(int argc, const char *argv[])
{
    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);

    // Initialize the robot.
    mbot_bridge::MBot robot;

    // *** Task: Drive in a five pointed star *** //

    // *** End student code *** //

    // Stop the robot.
    robot.stop();
    return 0;
}