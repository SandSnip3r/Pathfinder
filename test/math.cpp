#include "../math_helpers.h"

#include <gtest/gtest.h>

TEST(Pathfinder_Math, LessThan_Small) {
  EXPECT_TRUE(pathfinder::math::lessThan(0.0, 1.0));
  EXPECT_TRUE(pathfinder::math::lessThan(0.1, 1.0));
  EXPECT_TRUE(pathfinder::math::lessThan(0.5, 1.0));
  EXPECT_TRUE(pathfinder::math::lessThan(0.9, 1.0));
  EXPECT_TRUE(pathfinder::math::lessThan(0.99, 1.0));
  EXPECT_TRUE(pathfinder::math::lessThan(0.999, 1.0));
  EXPECT_TRUE(pathfinder::math::lessThan(-1.0, 0.0));
  EXPECT_TRUE(pathfinder::math::lessThan(-1.0, -0.1));
  EXPECT_TRUE(pathfinder::math::lessThan(-1.0, -0.5));
  EXPECT_TRUE(pathfinder::math::lessThan(-1.0, -0.9));
  EXPECT_TRUE(pathfinder::math::lessThan(-1.0, -0.99));
  EXPECT_TRUE(pathfinder::math::lessThan(-1.0, -0.999));
  
  EXPECT_FALSE(pathfinder::math::lessThan(1.0, 0.0));
  EXPECT_FALSE(pathfinder::math::lessThan(1.0, 0.1));
  EXPECT_FALSE(pathfinder::math::lessThan(1.0, 0.5));
  EXPECT_FALSE(pathfinder::math::lessThan(1.0, 0.9));
  EXPECT_FALSE(pathfinder::math::lessThan(1.0, 0.99));
  EXPECT_FALSE(pathfinder::math::lessThan(1.0, 0.999));
  EXPECT_FALSE(pathfinder::math::lessThan(0.0, -1.0));
  EXPECT_FALSE(pathfinder::math::lessThan(-0.1, -1.0));
  EXPECT_FALSE(pathfinder::math::lessThan(-0.5, -1.0));
  EXPECT_FALSE(pathfinder::math::lessThan(-0.9, -1.0));
  EXPECT_FALSE(pathfinder::math::lessThan(-0.99, -1.0));
  EXPECT_FALSE(pathfinder::math::lessThan(-0.999, -1.0));

  EXPECT_FALSE(pathfinder::math::lessThan(0.0, 0.0));
  EXPECT_FALSE(pathfinder::math::lessThan(1e-100, 1e-100));
  EXPECT_FALSE(pathfinder::math::lessThan(-1e-100, -1e-100));
}

TEST(Pathfinder_Math, LessThan_Large) {
  EXPECT_TRUE(pathfinder::math::lessThan(9999999999.0, 10000000000.0));
  EXPECT_TRUE(pathfinder::math::lessThan(99999999999.0, 100000000000.0));
  EXPECT_TRUE(pathfinder::math::lessThan(999999999999.0, 1000000000000.0));
  EXPECT_TRUE(pathfinder::math::lessThan(9999999999999.0, 10000000000000.0));
  EXPECT_TRUE(pathfinder::math::lessThan(99999999999999.0, 100000000000000.0));
  EXPECT_TRUE(pathfinder::math::lessThan(999999999999999.0, 1000000000000000.0));
  EXPECT_TRUE(pathfinder::math::lessThan(9.9e100, 1e101));
  EXPECT_TRUE(pathfinder::math::lessThan(-10000000000.0, -9999999999.0));
  EXPECT_TRUE(pathfinder::math::lessThan(-100000000000.0, -99999999999.0));
  EXPECT_TRUE(pathfinder::math::lessThan(-1000000000000.0, -999999999999.0));
  EXPECT_TRUE(pathfinder::math::lessThan(-10000000000000.0, -9999999999999.0));
  EXPECT_TRUE(pathfinder::math::lessThan(-100000000000000.0, -99999999999999.0));
  EXPECT_TRUE(pathfinder::math::lessThan(-1000000000000000.0, -999999999999999.0));
  EXPECT_TRUE(pathfinder::math::lessThan(-1e101, -9.9e100));

  EXPECT_FALSE(pathfinder::math::lessThan(10000000000.0, 9999999999.0));
  EXPECT_FALSE(pathfinder::math::lessThan(100000000000.0, 99999999999.0));
  EXPECT_FALSE(pathfinder::math::lessThan(1000000000000.0, 999999999999.0));
  EXPECT_FALSE(pathfinder::math::lessThan(10000000000000.0, 9999999999999.0));
  EXPECT_FALSE(pathfinder::math::lessThan(100000000000000.0, 99999999999999.0));
  EXPECT_FALSE(pathfinder::math::lessThan(1000000000000000.0, 999999999999999.0));
  EXPECT_FALSE(pathfinder::math::lessThan(1e101, 9.9e100));
  EXPECT_FALSE(pathfinder::math::lessThan(-9999999999.0, -10000000000.0));
  EXPECT_FALSE(pathfinder::math::lessThan(-99999999999.0, -100000000000.0));
  EXPECT_FALSE(pathfinder::math::lessThan(-999999999999.0, -1000000000000.0));
  EXPECT_FALSE(pathfinder::math::lessThan(-9999999999999.0, -10000000000000.0));
  EXPECT_FALSE(pathfinder::math::lessThan(-99999999999999.0, -100000000000000.0));
  EXPECT_FALSE(pathfinder::math::lessThan(-999999999999999.0, -1000000000000000.0));
  EXPECT_FALSE(pathfinder::math::lessThan(-9.9e100, -1e101));

  EXPECT_FALSE(pathfinder::math::lessThan(1e100, 1e100));
  EXPECT_FALSE(pathfinder::math::lessThan(-1e100, -1e100));
}

TEST(Pathfinder_Math, LineSegmentIntersectsWithCircle) {
  // int pathfinder::math::lineSegmentIntersectsWithCircle(Vector lineSegmentStartPoint, Vector lineSegmentEndPoint, Vector centerOfCircle, const double circleRadius, Vector *intersectionPoint1=nullptr, Vector *intersectionPoint2=nullptr);
  {
    // Horizontal line, left to right, on the x axis
    const pathfinder::Vector segmentStart{0.0, 0.0};
    const pathfinder::Vector segmentEnd{10.0, 0.0};
    const pathfinder::Vector circleCenter{5.0, 0.0};
    const double circleRadius{2.0};
    pathfinder::Vector ip1, ip2;
    int result = pathfinder::math::lineSegmentIntersectsWithCircle(segmentStart, segmentEnd, circleCenter, circleRadius, &ip1, &ip2);
    ASSERT_EQ(result, 2);
    EXPECT_EQ(ip1.x(), 3.0);
    EXPECT_EQ(ip1.y(), 0.0);
    EXPECT_EQ(ip2.x(), 7.0);
    EXPECT_EQ(ip2.y(), 0.0);
  }
  {
    // Horizontal line, right to left, on the x axis
    const pathfinder::Vector segmentStart{10.0, 0.0};
    const pathfinder::Vector segmentEnd{0.0, 0.0};
    const pathfinder::Vector circleCenter{5.0, 0.0};
    const double circleRadius{2.0};
    pathfinder::Vector ip1, ip2;
    int result = pathfinder::math::lineSegmentIntersectsWithCircle(segmentStart, segmentEnd, circleCenter, circleRadius, &ip1, &ip2);
    ASSERT_EQ(result, 2);
    EXPECT_EQ(ip1.x(), 7.0);
    EXPECT_EQ(ip1.y(), 0.0);
    EXPECT_EQ(ip2.x(), 3.0);
    EXPECT_EQ(ip2.y(), 0.0);
  }
  {
    // Vertical line, bottom to top, on the y axis
    const pathfinder::Vector segmentStart{0.0, 0.0};
    const pathfinder::Vector segmentEnd{0.0, 10.0};
    const pathfinder::Vector circleCenter{0.0, 5.0};
    const double circleRadius{2.0};
    pathfinder::Vector ip1, ip2;
    int result = pathfinder::math::lineSegmentIntersectsWithCircle(segmentStart, segmentEnd, circleCenter, circleRadius, &ip1, &ip2);
    ASSERT_EQ(result, 2);
    EXPECT_EQ(ip1.x(), 0.0);
    EXPECT_EQ(ip1.y(), 3.0);
    EXPECT_EQ(ip2.x(), 0.0);
    EXPECT_EQ(ip2.y(), 7.0);
  }
  {
    // Vertical line, top to bottom, on the y axis
    const pathfinder::Vector segmentStart{0.0, 10.0};
    const pathfinder::Vector segmentEnd{0.0, 0.0};
    const pathfinder::Vector circleCenter{0.0, 5.0};
    const double circleRadius{2.0};
    pathfinder::Vector ip1, ip2;
    int result = pathfinder::math::lineSegmentIntersectsWithCircle(segmentStart, segmentEnd, circleCenter, circleRadius, &ip1, &ip2);
    ASSERT_EQ(result, 2);
    EXPECT_EQ(ip1.x(), 0.0);
    EXPECT_EQ(ip1.y(), 7.0);
    EXPECT_EQ(ip2.x(), 0.0);
    EXPECT_EQ(ip2.y(), 3.0);
  }
  {
    // Line where one end of the segment is exactly on the circumference of the circle
    const pathfinder::Vector segmentStart{0.0, 0.0};
    const pathfinder::Vector segmentEnd{0.0, 10.0};
    const pathfinder::Vector circleCenter{0.0, 15.0};
    const double circleRadius{5.0};
    pathfinder::Vector ip1, ip2;
    int result = pathfinder::math::lineSegmentIntersectsWithCircle(segmentStart, segmentEnd, circleCenter, circleRadius, &ip1, &ip2);
    ASSERT_EQ(result, 1);
    EXPECT_EQ(ip1.x(), segmentEnd.x());
    EXPECT_EQ(ip1.y(), segmentEnd.y());
  }
  {
    // Line where one end of the segment is exactly on the circumference of the circle (the line passes through the circle)
    const pathfinder::Vector segmentStart{0.0, 0.0};
    const pathfinder::Vector segmentEnd{0.0, 10.0};
    const pathfinder::Vector circleCenter{0.0, 8.0};
    const double circleRadius{2.0};
    pathfinder::Vector ip1, ip2;
    int result = pathfinder::math::lineSegmentIntersectsWithCircle(segmentStart, segmentEnd, circleCenter, circleRadius, &ip1, &ip2);
    ASSERT_EQ(result, 2);
    EXPECT_EQ(ip1.x(), 0.0);
    EXPECT_EQ(ip1.y(), 6.0);
    EXPECT_EQ(ip2.x(), segmentEnd.x());
    EXPECT_EQ(ip2.y(), segmentEnd.y());
  }
  {
    // Horizontal line tangent to circle, touching bottom of circle
    const pathfinder::Vector segmentStart{0.0, 0.0};
    const pathfinder::Vector segmentEnd{10.0, 0.0};
    const pathfinder::Vector circleCenter{5.0, 5.0};
    const double circleRadius{5.0};
    pathfinder::Vector ip1, ip2;
    int result = pathfinder::math::lineSegmentIntersectsWithCircle(segmentStart, segmentEnd, circleCenter, circleRadius, &ip1, &ip2);
    ASSERT_EQ(result, 1);
    EXPECT_EQ(ip1.x(), circleCenter.x());
    EXPECT_EQ(ip1.y(), 0);
  }
  {
    // Vertical line tangent to circle, touching right of circle
    const pathfinder::Vector segmentStart{5.0, -10.0};
    const pathfinder::Vector segmentEnd{5.0, 10.0};
    const pathfinder::Vector circleCenter{0.0, 0.0};
    const double circleRadius{5.0};
    pathfinder::Vector ip1, ip2;
    int result = pathfinder::math::lineSegmentIntersectsWithCircle(segmentStart, segmentEnd, circleCenter, circleRadius, &ip1, &ip2);
    ASSERT_EQ(result, 1);
    EXPECT_EQ(ip1.x(), 5.0);
    EXPECT_EQ(ip1.y(), 0.0);
  }
}

TEST(Pathfinder_Math, Normalize) {
  struct TestData {
    double input;
    double expectedOutput;
  };

  constexpr const double kMax1{1.0};
  std::vector<TestData> testData1 { {    0.0,    0.0 },
                                    { 0.9999, 0.9999 },
                                    {    1.0,    0.0 },
                                    {   0.25,   0.25 },
                                    {    0.5,    0.5 },
                                    {   0.75,   0.75 },
                                    {   -1.0,    0.0 },
                                    {    2.0,    0.0 },
                                    {   -1.5,    0.5 }};
  for (const auto &data : testData1) {
    EXPECT_EQ(pathfinder::math::normalize(data.input, kMax1), data.expectedOutput);
  }

  constexpr const double kMax2{pathfinder::math::k2Pi};
  std::vector<TestData> testData2 { {                     0.0,                   0.0 },
                                    {                     1.0,                   1.0 },
                                    {   pathfinder::math::kPi, pathfinder::math::kPi },
                                    {  pathfinder::math::k2Pi,                   0.0 },
                                    {                    10.0,    3.7168146928204138 },
                                    {                   100.0,     5.752220392306214 },
                                    {                   -10.0,    2.5663706143591725 },
                                    {                  -100.0,    0.5309649148733797 },
                                    { -pathfinder::math::k2Pi,                   0.0 }};
  for (const auto &data : testData2) {
    EXPECT_DOUBLE_EQ(pathfinder::math::normalize(data.input, kMax2), data.expectedOutput);
  }
}