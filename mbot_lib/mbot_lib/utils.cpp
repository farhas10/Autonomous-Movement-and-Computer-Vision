/**
 * File: utils.cpp
 * 
 * Sources for functions for handling common tasks like working with geometry and interpreting lidar data.
 */

#include <mbot_lib/utils.h>


std::vector<float> rayConversionCartisean(float dist, float angle) 
{
    float x = dist * cos(angle);
    float y = dist * sin(angle);

    return std::vector<float> {x, y};
}

std::vector<float> rayConversionVector(float angle) 
{
    float vx = cos(angle);
    float vy = sin(angle);

    return std::vector<float> {vx, vy};
}

int findMinDist(const std::vector<float>& ranges)
{
    float minDist = std::numeric_limits<float>::max(); //setting the minimum distance to max value of distance float value in the vector container
    int minIndex = -1;
    for(int i = 0; i < ranges.size(); i++){
        if(ranges[i] < minDist) {
            minDist = ranges[i];
            minIndex = static_cast<int>(i); //converting the index to an integer and assigning it to the minIndex variable
        }
    }
    return minIndex; //returns -1 (initialised value if no valid minimum was found), otherwise, it returns the index of the minimum value found
}

int findMinNonzeroDist(const std::vector<float>& ranges)
{
    float minDist = std::numeric_limits<float>::max();
    float minIndex = -1;
    for(int i = 0; i < ranges.size(); i++){
        if((ranges[i] > 0) && (ranges[i] < minDist)){ //additional condition that the value of element in ranges is not zero
            minDist = ranges[i];
            minIndex = static_cast<int>(i);
        }
    }
    return minIndex;
}

std::vector<float> vectorAdd(const std::vector<float>& v1, const std::vector<float>& v2) 
{
    // *** Task: Implement this function according to the header file *** //
    float vx; float vy; float vr;
    std::vector<float> added {vx, vy, vr};
    vx = v1[0] + v2[0];
    vy = v1[1] + v2[1];
    vr = v1[2] + v2[2];
    return added;

    // *** End student code *** //
}

std::vector<float> crossProduct(const std::vector<float>& v1, const std::vector<float>& v2) 
{
    // *** Task: Implement this function according to the header file *** //
    float vx; float vy; float vr;
    std::vector<float> crossproduct {vx, vy, vr};
    vx = (v1[1]*v1[2])-(v1[2]*v2[1]);
    vy = -1*(v1[0]*v2[2])-(v1[2]*v2[0]);
    vr = (v1[0]*v2[1])-(v1[1]*v2[0]);
    return crossproduct;

    // *** End student code *** //
}

void transformVector2D(std::vector<float>& xy, float theta) 
{
    float x = xy[0] * cos(theta) - xy[1] * sin(theta);
    float y = xy[0] * sin(theta) + xy[1] * cos(theta);
    
    xy[0] = x;
    xy[1] = y;
}
