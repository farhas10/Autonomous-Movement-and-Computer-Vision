/**
 * File: follow_1d.cpp
 * Controls the robot to maintain a given distance to the wall directly in front of it.
 * This code uses Bang-Bang control or P-control to maintain the setpoint distance.
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

    mbot_bridge::MBot robot;
    std::vector<float> ranges;
    std::vector<float> thetas;

    float setpoint = 0.4; 
    float tolerance = 0.1;
    float scaling = -0.5;
    float kp = 0.5;

    while (true) {
        robot.readLidarScan(ranges, thetas);

        float dist_to_wall = findFwdDist(ranges, thetas);
        if (dist_to_wall < 0) continue;

        std::cout << "Distance to wall: " << dist_to_wall << std::endl;

        //float controlSignal = pControl(dist_to_wall, setpoint, kp)
        float controlSignal = bangBangControl(dist_to_wall, setpoint, scaling, tolerance);
        std::cout << "Control Signal: " << controlSignal << std::endl;  // Debugging output
        robot.drive(controlSignal, 0.0, 0.0);

        if (ctrl_c_pressed)
            break;
    }

    // Stop the robot.
    robot.stop();
    return 0;
}