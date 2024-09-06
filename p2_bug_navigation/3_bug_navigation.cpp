#include <cmath>
#include <iostream>
#include <signal.h>

#include <mbot_bridge/robot.h>

#include <mbot_lib/behaviors.h>
#include <mbot_lib/controllers.h>
#include <mbot_lib/utils.h>


bool ctrl_c_pressed;
void ctrlc(int) {
    ctrl_c_pressed = true;
}


int main() {
    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);

    // Initialize the robot.
    mbot_bridge::MBot robot;
    // Reset the robot odometry to zero.
    robot.resetOdometry();

    // *** Task: Get the goal pose (x, y, theta) from the user *** //

    // *** End student code *** //

    // *** Task: Implement bug navigation finite state machine *** //
    
    // NOTE: You may want to change the condition in this loop.
    while (true) {

        if(ctrl_c_pressed) break;
    }

    // *** End student code *** //

    // Stop the robot.
    robot.stop();

    // *** Task: Print out the robot's final odometry pose *** //
    
    // *** End student code *** //
}
