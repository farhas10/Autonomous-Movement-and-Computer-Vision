#include <iostream>
#include <cmath>

#include <mbot_bridge/robot.h>

#include <mbot_lib/behaviors.h>
#include <mbot_lib/controllers.h>
#include <mbot_lib/utils.h>


bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


int main(){
    // Initialize the robot.
    mbot_bridge::MBot robot;
    // Reset the robot odometry to zero at the beginning of the run.
    robot.resetOdometry();

    // *** Task: Get the goal pose (x, y, theta) from the user *** //

    // *** End student code *** //
    
    while (true) {
        
        // *** Task: Implement hit the spot *** //

        // *** End student code *** //
        
        if (ctrl_c_pressed) break;
    }

    // Stop the robot before exiting.
    robot.stop();

    // *** Task: Print out the robot's final odometry pose *** //
    
    // *** End student code *** //

    return 0;
}
