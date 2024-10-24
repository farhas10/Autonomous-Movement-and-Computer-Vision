#include <iostream>
#include <cmath>

#include <mbot_bridge/robot.h>

#include <mbot_lib/behaviors.h>
#include <mbot_lib/controllers.h>
#include <mbot_lib/utils.h>
using namespace std;

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
    float x; float y; float theta;
    cout << "Enter your X coordinate: " << "\n";
    cin >> x;
    cout << "Enter your Y coordinate: " << "\n";
    cin >> y;
    cout << "Enter your desired angle (degrees): " << "\n";
    cin >> theta;

    std::vector<float> goal = {x, y, theta};
    // *** End student code *** //
    
    while (true) {
        // *** Task: Implement hit the spot *** //
        std::vector<float> coords = computeDriveToPoseCommand(goal, robot.readOdometry());
        robot.drive(coords[0], coords[1], coords[2]);
        // *** End student code *** //
        
        if (ctrl_c_pressed) break;
    }

    // Stop the robot before exiting.
    robot.stop();

    std::vector<float> finalPose = robot.readOdometry();
    std::cout << "Final MBot odometry x-pose = " << finalPose[0] 
              << ", y = " << finalPose[1] 
              << ", theta = " << finalPose[2] 
              << std::endl;

    return 0;
}
