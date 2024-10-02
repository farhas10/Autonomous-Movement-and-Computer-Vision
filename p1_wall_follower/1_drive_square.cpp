/**
 * File: drive_square.cpp
 *
 * Code to drive in a square N times.
 */

#include <iostream>
#include <cmath>
#include <mbot_bridge/robot.h>
#include <mbot_lib/utils.h>


int main(int argc, const char *argv[])
{
    // Initialize the robot.
    mbot_bridge::MBot robot;

    // Variables to be tuned.
    float vel = 0.5;
    float dt = 1.0;
    int num_square = 3;

    // Repeating the square movements depending on the number of squares desired.
    for(int i = 0; i < num_square; ++i){
            //Will move the robot 0.5 m/s in the x direction.
            robot.drive(0.5, 0.0, 0.0);
            //Will execute for one second.
            sleep(1);
            //Same thing but in Y direction.
            robot.drive(0.0, 0.5, 0.0);
            sleep(1);
            robot.drive(-0.5, 0.0, 0.0);
            sleep(1);
            robot.drive(0.0, -0.5, 0.0);
            sleep(1);
    }
    
    //Robot stops moving.
    std::cout << "Stopping the robot!!" << std::endl;
    robot.stop();
    return 0;
}
