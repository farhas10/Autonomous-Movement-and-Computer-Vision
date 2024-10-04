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
    vector<float> xyPose = {pose[0], pose[1]};
    vector<float> xyGoal = {goal[0], goal[1]};

    transformVector2D(xyPose, pose[2]);
    transformVector2D(xyGoal, goal[2]);
    
    xyGoal[0] *= -1;
    xyGoal[1] *= -1;
    xyGoal[2] *= -1;

    
    //Computing the resultant vector.
    vector<float> result = vectorAdd(xyPose, xyGoal);

    float magnitude = pow((result[0] * result[0]) + (result[1]*result[1]),1/2);

    result[0] *= result[0]/magnitude;
    result[1] *= result[1]/magnitude;

    return result;

    // *** End student code *** //
}

bool isGoalAngleObstructed(const std::vector<float>& goal, const std::vector<float>& pose,
                           const std::vector<float>& ranges, const std::vector<float>& thetas)
{
    // *** Task: Implement this function according to the header file *** //

    // *** End student code *** //
}