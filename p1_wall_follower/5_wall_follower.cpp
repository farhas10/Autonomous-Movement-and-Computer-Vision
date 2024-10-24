/**
 * File: wall_follower.cpp
 *
 * Controls robot to follow a wall. 
 */
#include <iostream>
#include <cmath>
#include <signal.h>
#include <mbot_bridge/robot.h>
#include <mbot_lib/behaviors.h>
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
    // Create empty vectors to store the scan data.
    std::vector<float> ranges;
    std::vector<float> thetas;

   
    while (true) {

        //Reading the surrounding environment into arrays.
        robot.readLidarScan(ranges, thetas);
         
         //Computing directional cross product through wall follower command.
         std::vector<float> vals = computeWallFollowerCommand(ranges, thetas);

         //Entering x, y, and angular parameters for robot to drive.
         robot.drive(vals[0], vals[1], vals[2]);
        
        if (ctrl_c_pressed) break;
    }
    // Stop the robot.
    robot.stop();
    return 0;
}
