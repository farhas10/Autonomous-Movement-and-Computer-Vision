/**
 * File: follow_2d.cpp
 *
 * Controls the robot to maintain a given distance to the nearest wall.
 *
 * This code finds the distance to the nearest wall in the Lidar scan. It
 * applies a control to the robot in the direction of the wall using the angle
 * to the scan.
 */

#include <cmath>
#include <iostream>

#include <signal.h>

#include <mbot_bridge/robot.h>

#include <mbot_lib/controllers.h>
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

    // We will store the LiDar scan data in these vectors.
    std::vector<float> ranges;
    std::vector<float> thetas;

    // *** Task 1: Adjust these values appropriately ***

    float setpoint = 0.5;  // The goal distance from the wall in meters

    // *** End student code *** //
    while (true) {
        // This function gets the Lidar scan data.
        robot.readLidarScan(ranges, thetas);

        // Get the distance to the wall.
        float min_idx = findMinNonzeroDist(ranges);
        float dist_to_wall = ranges[min_idx];
        float angle_to_wall = thetas[min_idx];

        // *** Task 2: Implement the 2D Follow Me controller ***
        // Hint: Look at your code from follow_1D
        // Hint: When you compute the velocity command, you might find the functions
        // rayConversionVector helpful!
        
        float error = setpoint - dist_to_wall; 
        std::vector<float> directionVector = rayConversionVector(angle_to_wall);

        //Angular and linear constants.
        float k_linear = 1.0;
        float k_angular = 2.0;

        //Corresponding velocity constants.
        float linear_velocity = k_linear * error;
        float angular_velocity = k_angular * (error/setpoint);
        
        //Drive in the direction of the vector.
        robot.drive(pControl(directionVector[0], linear_velocity, error),pControl(directionVector[1], linear_velocity, error), 0);
        
        //Command stopping the robot.
        if (ctrl_c_pressed) break;
    }

    // Stop the robot.
    robot.stop();
    return 0;
}


