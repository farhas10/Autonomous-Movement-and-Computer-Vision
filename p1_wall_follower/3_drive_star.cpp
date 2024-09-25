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
    const float dist = 2.0;
    const float angle = 144.0;
    for(int i = 0; i < 5; i++) {
        std::vector<float> cartesianCoords rayConversionCartisean(dist, angle * (i+1))
        std::cout << "The Cartesian Coordinates for point " << i << "is (" << cartesianCoords[0] << ", " << cartesianCoords[1] << ")\n";
        robot.moveForward(dist);
        std::vector<float> directionVector rayConversionVector(angle * (i+1));
        std::cout << "The Direction Vector for point " << i << "is (" << directionVector[0] << ", " << directionVector[1] << ")\n";
        robot.turn(angle);
    }

    // Stop the robot
    robot.stop();
    return 0;
}