/**
 * File: behaviors.cpp
 * 
 * Sources for high level functions that determine how the robot should move.
 */

#include <mbot_lib/behaviors.h>
using namespace std;

std::vector<float> computeWallFollowerCommand(const std::vector<float>& ranges, const std::vector<float>& thetas)
{
    //All necessary constants
    float setpoint = 0.4;
    float velocity = 0.5;
    float kp = 0.5;

    //Finding angles and distances to wall.
    int minIndex = findMinNonzeroDist(ranges);
    float dist_to_wall = ranges[minIndex];
    float angle_to_wall = thetas[minIndex];

    //Result vector created.
    std::vector<float> result = {0,0,0};

    //Finding diretion vectors.
    result = rayConversionVector(angle_to_wall);

    //Arbitrary z-axis vector/
    std::vector<float> zaxis = {0,0,1};

    //Cross product computation.
    std::vector<float> crossPVelocity = crossProduct(result, zaxis);

    float vy = velocity*crossPVelocity[0];
    float vx = velocity*crossPVelocity[1];

    float error = pControl(dist_to_wall, setpoint, kp*-1);

    //Adjustment based on distances using PControl.
    vx += error*result[0];
    vy += error*result[1];

    std::vector<float> drive = {vx, vy, 0};
    return drive; 

    // *** End student code *** //
}

std::vector<float> computeDriveToPoseCommand(const std::vector<float>& goal, const std::vector<float>& pose)
{   
    // *** Task: Implement this function according to the header file *** //
    
    //Converting to usable coordinates rotated by theta.
    vector<float> origin = transformVector2D(pose);
    vector<float> target = transformVector2D(goal);
    
    //Computing the resultant vector.
    vector<float> result = origin + target;
    result[0] *= bangBangControl(pose[0], goal[0], 0.75, 0.01);
    result[1] *= bangBangControl(pose[1], goal[1], 0.75, 0.01);

    return result;

    // *** End student code *** //
}

bool isGoalAngleObstructed(const std::vector<float>& goal, const std::vector<float>& pose,
                           const std::vector<float>& ranges, const std::vector<float>& thetas)
{
    // *** Task: Implement this function according to the header file *** //
    robot.readLidarScan();
    return false;

    // *** End student code *** //
}