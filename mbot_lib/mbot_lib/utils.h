/**
 * File: utils.h
 *
 * Definitions for functions for handling common tasks like working with geometry and interpreting lidar data.
 */

#ifndef MBOT_LIB_UTILS_H
#define MBOT_LIB_UTILS_H

#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

#define HIGH 1e6

/**
 * Gets the current time in microseconds.
 * @return  The current time in microseconds.
 */
static inline int getTimeMicro()
{
    auto now = std::chrono::system_clock::now();
    return now.time_since_epoch().count();
}

/**
 * Sleeps for the provided number of seconds.
 * @param  secs Number of seconds to sleep for.
 */
static inline void sleepFor(const double secs)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(secs * 1000)));
}

/**
 * Normalizes angle to range [-PI, PI].
 * @param  angle Angle to normalize.
 * @return  Normalized angle.
 */
static inline double normalizeAngle(const double angle)
{
    const double result = fmod(angle + M_PI, 2.0 * M_PI);
    if(result <= 0) return result + M_PI;
    return result - M_PI;
}

/**
 * Finds the distance to the nearest obstacle directly in front of the robot.
 * @param  ranges The ranges from a lidar scan.
 * @param  thetas The angles from a lidar scan.
 * @return  The distance to the nearest obstacle directly in front of the robot.
 */
static float findFwdDist(const std::vector<float>& ranges, const std::vector<float>& thetas)
{
    // Simple average filter over the front rays in case a few are invalid.
    int num_range = 5;
    int num_scans = 0;
    float sum = 0;

    for (int i = 0; i < num_range; ++i)
    {
        if (ranges[i] > 0)
        {
            sum += ranges[i] * cos(thetas[i]);
            num_scans++;
        }
    }
    for (int i = ranges.size() - num_range; i < ranges.size(); ++i)
    {
        if (ranges[i] > 0)
        {
            sum += ranges[i] * cos(thetas[i]);
            num_scans++;
        }
    }

    if (num_scans < 1) return -1;

    return sum / num_scans;
}

/**
 * Finds the index of the minimum range in the given lidar scan, in a given angle range. Skips
 * values of zero which indicate bad readings. 
 * @param  ranges Vector of ray ranges.
 * @param  thetas Vector of ray angles.
 * @param  target_angle The angle the slice is centered around.
 * @param  slice_size The size of the slice.
 * @return  The index (location) of the smallest value in ranges within the slice.
 */
static int findMinNonzeroDistInSlice(const std::vector<float>& ranges, const std::vector<float>& thetas,
                                     float target_angle, float slice_size)
{
    // Iterate through every scan.
    int min_idx = -1;
    float min_dist = HIGH;
    for (int i = 0; i < thetas.size(); i++) {
        // Only look at scans within the view angle.
        if(abs(normalizeAngle(target_angle - thetas[i])) < slice_size) {
            if(ranges[i] < min_dist && ranges[i] > 0) {
                min_dist = ranges[i];
                min_idx = i;
            }
        }
    }

    return min_idx;
}

/**
 * Converts the distance and angle into x and y cartisean coords.
 * @param  dist Distance to point
 * @param  angle Angle to point
 * @return  Vector containing {x,y}
 */
std::vector<float> rayConversionCartisean(float dist, float angle);

/**
 * Converts angle to object into vector to object.
 * @param  angle Angle to point
 * @return  Vector of length 3 containing {x,y,z}
 */
std::vector<float> rayConversionVector(float angle);

/**
 * Finds the index of the minimum distance in the given vector of Lidar ranges.
 * @param  ranges Vector of floats.
 * @return  The index (location) of the smallest value in ranges.
 * 
 * NOTE: This function will cause trouble because it does not skip bad Lidar readings. 
 *       See findMinNonzeroDist() for a version that works well with Lidar. 
 */
int findMinDist(const std::vector<float>& ranges);

/**
 * Finds the index of the minimum distance in the given vector of Lidar ranges,
 * skipping values of zero, which indicate bad readings.
 * @param  ranges Vector of floats.
 * @return  The index (location) of the smallest value in ranges.
 */
int findMinNonzeroDist(const std::vector<float>& ranges);

/**
 * Adds two vectors.
 * @param  v1 First vector.
 * @param  v2 Second vector.
 * @return  Sum of v1 and v2.
 */
std::vector<float> vectorAdd(const std::vector<float>& v1, const std::vector<float>& v2);

/**
 * Performs a cross product between two 3D vectors.
 * @param  v1 First vector.
 * @param  v2 Second vector.
 * @return  Cross product between v1 and v2.
 */
std::vector<float> crossProduct(const std::vector<float>& v1, const std::vector<float>& v2);

/**
 * Transforms a 2D vector into new coordinate frame that is rotated theta degrees 
 * counter clockwise from the coordinate frame in which it is currently represented.
 * @param  xy A 2D vector.
 * @param  theta The angle to rotate the vector (radians).
 */
void transformVector2D(std::vector<float>& xy, float theta);

#endif  // MBOT_LIB_UTILS_H