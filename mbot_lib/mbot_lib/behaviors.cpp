/**
 * File: behaviors.cpp
 * 
 * Sources for high level functions that determine how the robot should move.
 */

#include <mbot_lib/behaviors.h>


std::vector<float> computeWallFollowerCommand(const std::vector<float>& ranges, const std::vector<float>& thetas)
{
    float setpoint = 0.5;
    float k_linear = 1.0;
    float k_angular = 2.0;

    int minIndex = findMinNonzeroDist(ranges);
    float dist_to_wall = ranges[minIndex];
    float angle_to_wall = thetas[minIndex];

    float error = setpoint - dist_to_wall;

    float linear_velocity = k_linear * error;
    float angular_velocity = k_angular * (error/setpoint);
    
    return std::vector<float>{linear_velocity, 0.0, angular_velocity};

    // *** End student code *** //
}

std::vector<float> computeDriveToPoseCommand(const std::vector<float>& goal, const std::vector<float>& pose)
{   
    // *** Task: Implement this function according to the header file *** //

    return std::vector<float>();

    // *** End student code *** //
}

bool isGoalAngleObstructed(const std::vector<float>& goal, const std::vector<float>& pose,
                           const std::vector<float>& ranges, const std::vector<float>& thetas)
{
    // *** Task: Implement this function according to the header file *** //

    return false;

    // *** End student code *** //
}