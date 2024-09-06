#include <gtest/gtest.h>

#include <mbot_lib/behaviors.h>
#include <mbot_lib/controllers.h>
#include <mbot_lib/utils.h>
#include <super_mario.h>
#include <test_utils.h>

TEST(BangBangControl, TestZero) {
    float current_point = 1;
    float setpoint = 1;
    float scaling = 1;
    float result = bangBangControl(current_point,setpoint,scaling);
    ASSERT_EQ(result, 0);
}

TEST(BangBangControl, TestNearZero) {
    float current_point = 1;
    float setpoint = 1.01;
    float scaling = .2;
    float tolerance = .1;
    float result = bangBangControl(current_point,setpoint,scaling, tolerance);
    ASSERT_EQ(result, 0);
}

TEST(BangBangControl, TestPositive) {
    float current_point = 1;
    float setpoint = 2;
    float scaling = .5;
    float tolerance = .1;
    float result = bangBangControl(current_point,setpoint,scaling, tolerance);
    ASSERT_EQ(result, .5);
}

TEST(BangBangControl, TestNegative) {
    float current_point = 1;
    float setpoint = 0;
    float scaling = .5;
    float tolerance = .1;
    float result = bangBangControl(current_point,setpoint,scaling, tolerance);
    ASSERT_EQ(result, -.5);
}

TEST(PControl, TestZero) {
    float current = 1;
    float setpoint =1;
    float kp= .5;
    float result = pControl(current,setpoint,kp);
    ASSERT_EQ(result, 0.0);
}

TEST(PControl, TestNegative) {
    float current = 1;
    float setpoint = .5;
    float kp= .3;
    float result = pControl(current,setpoint,kp);
    ASSERT_FLOAT_EQ(result, (.5 - 1) * .3);
}

TEST(PControl, TestPositive) {
    float current = 1;
    float setpoint = 2;
    float kp= .2;
    float result = pControl(current,setpoint,kp);
    ASSERT_FLOAT_EQ(result, (1) * .2);
}

TEST(RayConversionCartisean, TestOne) {
    float dist = 10;
    float angle = 1.5;
    std::vector<float> correct_result = {.707372, 9.97495};
    testRayConversionCartesian(dist, angle, correct_result);
}

TEST(RayConversionCartisean, TestTwo) {
    float dist = 5;
    float angle = -0.52;
    std::vector<float> correct_result = {4.3390961,-2.4844};
    testRayConversionCartesian(dist, angle, correct_result);
}

TEST(RayConversionVector, TestPos) {
    float angle = .2;
    std::vector<float> correct_result = {0.9800666, 0.19866933, 0};
    testRayConversionVector(angle,correct_result);
}

TEST(RayConversionVector, TestNegative) {
    float angle = -.2;
    std::vector<float> correct_result = {0.9800666,  -0.19866933, 0};
    testRayConversionVector(angle,correct_result);
}

TEST(RayConversionVector, TestZero) {
    float angle = 0;
    std::vector<float> correct_result = {1, 0, 0};
    testRayConversionVector(angle,correct_result);
}

TEST(FindMinDist, TestBasic) {
    std::vector<float> v1 = {0.5, 200, 12.6, 0.2, 0.3, 10.4, 61, 0.99, 1.5};
    int result = findMinDist(v1);
    ASSERT_EQ(result, 3);
}

TEST(FindMinNonzeroDist, TestBasic) {
    std::vector<float> v1 = {0.5, 200, 12.6, 0.2, 0.3, 10.4, 61, 0.99, 1.5};
    int result = findMinDist(v1);
    ASSERT_EQ(result, 3);
}

TEST(FindMinNonzeroDist, TestIgnoreZero) {
    std::vector<float> v2 = {9, 7, 5, 3, 1, 0, 2, 4, 6, 8};
    int result = findMinNonzeroDist(v2);
    ASSERT_EQ(result, 4);
}

TEST(FindMinNonzeroDist, TestIgnoreFirstZero) {
    std::vector<float> v3 = {0, 0.01, 1, 0.011, 10, 1};
    int result = findMinNonzeroDist(v3);
    ASSERT_EQ(result, 1);
}

TEST(FindMinNonzeroDist, TestIgnoreFirstZeros) {
    std::vector<float> v4 = {0, 0, 0, 0.01, 1, 0.011, 10, 1};
    int result = findMinNonzeroDist(v4);
    ASSERT_EQ(result, 3);
}

TEST(VectorAdd, TestBasic) {
    std::vector<float> v1,v2,correct_result;
    v1 = {1,2.5,1.2};
    v2 = {1,.5,.3};
    correct_result = {2,3,1.5};
    testVectorAdd(v1,v2,correct_result);
}

TEST(CrossProduct, TestBasic) {
    std::vector<float> v3 = {0.5, -1.2, 0.7};
    std::vector<float> v1 = {0, 0, 1};
    std::vector<float> correct_result = {-1.2, -0.5, 0};
    testCrossProduct(v3, v1, correct_result);
}

TEST(SuperMario, TestNoEvents) {
    testSuperMario("../test/no_events.txt", 40);
}

TEST(SuperMario, TestSequence1) {
    testSuperMario("../test/seq01.txt", 48);
}

TEST(SuperMario, TestSequence2) {
    testSuperMario("../test/seq02.txt", 102);
}

TEST(SuperMario, TestSequence3) {
    testSuperMario("../test/seq03.txt", 10);
}

TEST(TransformVector2D, TestZeroTheta) {
    std::vector<float> input = {0, 1, 0};
    std::vector<float> output = {0, 1, 0};
    testTransformVector(input, 0, output);
}   

TEST(TransformVector2D, TestOne) {
    std::vector<float> input = {0, 1, 0};
    std::vector<float> output = {0.86602539, -0.50000006, 0};
    testTransformVector(input, 2 * M_PI / 3, output);
}

TEST(TransformVector2D, TestTwo) {
    std::vector<float> input = {1, 0, 0};
    std::vector<float> output = {0.809017, -0.58778524, 0};
    testTransformVector(input, M_PI / 5, output);
}