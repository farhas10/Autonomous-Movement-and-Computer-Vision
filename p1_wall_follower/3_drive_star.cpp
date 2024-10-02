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

    //Vector containing various angles for the robot to travel against. 
    std::vector<double> angle = {(0.0), ((-144) * M_PI / 180), ((72) * M_PI / 180), ((-108) * M_PI /180), ((144) * M_PI /180)};

    //Vector containing x and y direction vectors based on angles.
    std::vector<float> result;

    //Will loop through the different angles to travel.
    for(int i = 0; i < 5; ++i) {

        //Converts the angles to usable directions.
        result = rayConversionVector(angle[i]);

        //Driving in the corresponding direction.
        robot.drive(result[0] * 0.5 , result[1] * 0.5, 0);
        sleep(1);
        
    }

    // Stop the robot
    robot.stop();
    return 0;
}