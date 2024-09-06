#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <iostream>
#include <vector>

#include <gtest/gtest.h>

#include <mbot_lib/utils.h>
#include <super_mario.h>

void testCrossProduct(const std::vector<float>& v1, const std::vector<float>& v2,
                      const std::vector<float>& correct_result) {
    std::vector<float> result = crossProduct(v1, v2);
    ASSERT_EQ(result.size(), correct_result.size());
    for (int i = 0; i < result.size(); ++i) {
        ASSERT_FLOAT_EQ(result[i], correct_result[i]);
    }
}

void testRayConversionCartesian(float dist, float angle, const std::vector<float>& correct_result) {
    std::vector<float> result = rayConversionCartisean(dist, angle);
    ASSERT_EQ(result.size(), correct_result.size());
    ASSERT_FLOAT_EQ(result[0], correct_result[0]);
    ASSERT_FLOAT_EQ(result[1], correct_result[1]);
}

void testRayConversionVector(float angle, std::vector<float>& correct_result) {
    std::vector<float> result = rayConversionVector(angle);
    ASSERT_EQ(result.size(), correct_result.size());
    ASSERT_FLOAT_EQ(result[0], correct_result[0]);
    ASSERT_FLOAT_EQ(result[1], correct_result[1]);
    ASSERT_FLOAT_EQ(result[2], correct_result[2]);
}

void testVectorAdd(std::vector<float>& v1, std::vector<float>& v2, std::vector<float>& correct_result) {
    std::vector<float> result = vectorAdd(v1,v2);
    ASSERT_EQ(correct_result.size(), result.size());
    for(int i =0; i < result.size(); ++i) {
        ASSERT_FLOAT_EQ(result[i], correct_result[i]);
    }
}

void testSuperMario(const std::string &file_path, const int correct_score) {
    std::vector<char> events = readCourseData(file_path);
    int score = playGame(events, false);
    ASSERT_EQ(score, correct_score);
}

void testTransformVector(std::vector<float> &input, float theta, const std::vector<float> &correct_output) {
    transformVector2D(input, theta);
    ASSERT_FLOAT_EQ(input[0], correct_output[0]);
    ASSERT_FLOAT_EQ(input[1], correct_output[1]);
}

#endif  // TEST_UTILS_H
