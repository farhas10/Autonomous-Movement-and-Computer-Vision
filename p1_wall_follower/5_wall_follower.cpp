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
/**
    while (true) {

        std::vector<float> WallFollower computeWallFollower(ranges, thetas);

        float vx = WallFollower[0];
        float vy = WallFollower[1];
        float wtheta = WallFollower[2];

        robot.setVelocity(vx, wtheta);


        if (ctrl_c_pressed) break;
    }

    // Stop the robot.

    */
    robot.stop();
    return 0;
}
