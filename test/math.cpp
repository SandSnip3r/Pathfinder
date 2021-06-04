#include "../math_helpers.h"

#include <gtest/gtest.h>

#include <array>
#include <utility>

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

TEST(Pathfinder_Math, Intersect) {
  const std::vector<pathfinder::Vector> vertices{{ 0.0, -5.0}, // 0
                                                 { 0.0,  5.0}, // 1
                                                 {-5.0,  0.0}, // 2
                                                 { 5.0,  0.0}, // 3

                                                 {-2.0,  1.0}, // 4
                                                 {-1.0,  2.0}, // 5

                                                 {-7.0,  1.0}, // 6
                                                 {-6.0,  2.0}, // 7

                                                 {-2.0,  2.0}, // 8
                                                 {-7.0,  2.0}, // 9

                                                 { 0.0, -6.0}, // 10
                                                 { 1.0, -5.0}, // 11
                                                 { 1.0, -4.0}, // 12
                                                 { 0.0, -4.0}  // 13
                                                };
  pathfinder::Vector resultingIntersectionPoint;
  {
    // plus-shaped intersection
    const auto result = pathfinder::math::intersect(vertices[0], vertices[1], vertices[2], vertices[3], &resultingIntersectionPoint);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(resultingIntersectionPoint.x(), 0.0);
    ASSERT_EQ(resultingIntersectionPoint.y(), 0.0);
  }
  {
    // Intersection of self, horizonal
    const auto result = pathfinder::math::intersect(vertices[2], vertices[3], vertices[2], vertices[3]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    // Intersection of self, vertical
    const auto result = pathfinder::math::intersect(vertices[0], vertices[1], vertices[0], vertices[1]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    // Intersection of self, diagonal
    const auto result = pathfinder::math::intersect(vertices[0], vertices[3], vertices[0], vertices[3]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    const auto result = pathfinder::math::intersect(vertices[2], vertices[3], vertices[4], vertices[5]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersect(vertices[2], vertices[3], vertices[6], vertices[7]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersect(vertices[2], vertices[3], vertices[8], vertices[5]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersect(vertices[2], vertices[3], vertices[9], vertices[7]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersect(vertices[2], vertices[3], vertices[9], vertices[5]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersect(vertices[1], vertices[0], vertices[0], vertices[10], &resultingIntersectionPoint);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(vertices[0], resultingIntersectionPoint);
  }
  {
    const auto result = pathfinder::math::intersect(vertices[1], vertices[0], vertices[10], vertices[0], &resultingIntersectionPoint);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(vertices[0], resultingIntersectionPoint);
  }
  {
    const auto result = pathfinder::math::intersect(vertices[1], vertices[0], vertices[0], vertices[11], &resultingIntersectionPoint);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(vertices[0], resultingIntersectionPoint);
  }
  {
    const auto result = pathfinder::math::intersect(vertices[1], vertices[0], vertices[0], vertices[12], &resultingIntersectionPoint);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(vertices[0], resultingIntersectionPoint);
  }
  {
    const auto result = pathfinder::math::intersect(vertices[1], vertices[0], vertices[0], vertices[13]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    const auto result = pathfinder::math::intersect(vertices[1], vertices[0], vertices[13], vertices[0]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    // Colinear but segments are not touching
    const auto result = pathfinder::math::intersect(vertices[9], vertices[7], vertices[8], vertices[5]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    // Line 2 is 0-length and on line 1
    const auto result = pathfinder::math::intersect(vertices[10], vertices[13], vertices[0], vertices[0], &resultingIntersectionPoint);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(vertices[0], resultingIntersectionPoint);
  }
  {
    // Line 2 is 0-length and not on line 1 but is colinear with line 1
    const auto result = pathfinder::math::intersect(vertices[10], vertices[13], vertices[1], vertices[1]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    // Line 2 is 0-length and not on nor colinear with line 1
    const auto result = pathfinder::math::intersect(vertices[10], vertices[13], vertices[11], vertices[11]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    // Line 1 is 0-length and on line 2
    const auto result = pathfinder::math::intersect(vertices[0], vertices[0], vertices[10], vertices[13], &resultingIntersectionPoint);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(vertices[0], resultingIntersectionPoint);
  }
  {
    // Line 1 is 0-length and not on line 2 but is colinear with line 2
    const auto result = pathfinder::math::intersect(vertices[1], vertices[1], vertices[10], vertices[13]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    // Line 1 is 0-length and not on nor colinear with line 2
    const auto result = pathfinder::math::intersect(vertices[11], vertices[11], vertices[10], vertices[13]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    // Both lines are 0-length and not the same
    const auto result = pathfinder::math::intersect(vertices[0], vertices[0], vertices[1], vertices[1]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    // All 4 points are the same
    const auto result = pathfinder::math::intersect(vertices[0], vertices[0], vertices[0], vertices[0], &resultingIntersectionPoint);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(vertices[0], resultingIntersectionPoint);
  }
}

TEST(Pathfinder_Math, IsPointOnTriangle) {
  const std::vector<pathfinder::Vector> vertices{{  0.0,  0.0}, // 0
                                                 {  4.0,  0.0}, // 1
                                                 {  4.0,  4.0}, // 2
                                                 {  2.0,  0.0}, // 3
                                                 {  2.0,  2.0}, // 4
                                                 {  4.0,  2.0}, // 5
                                                 {  3.0,  1.0}, // 6
                                                 {  2.0, -1.0}, // 7
                                                 {  1.0,  3.0}, // 8
                                                 {  5.0,  2.0}, // 9
                                                 { -2.0, -1.0}, // 10
                                                 {  5.0,  6.0}, // 11
                                                 {  5.0, -1.0}, // 12
                                                };
  // Point is inside the triangle
  EXPECT_TRUE(pathfinder::math::isPointOnTriangle(vertices[6], vertices[0], vertices[1], vertices[2]));

  // Point is outside the triangle
  EXPECT_FALSE(pathfinder::math::isPointOnTriangle(vertices[7], vertices[0], vertices[1], vertices[2]));
  EXPECT_FALSE(pathfinder::math::isPointOnTriangle(vertices[8], vertices[0], vertices[1], vertices[2]));
  EXPECT_FALSE(pathfinder::math::isPointOnTriangle(vertices[9], vertices[0], vertices[1], vertices[2]));
  EXPECT_FALSE(pathfinder::math::isPointOnTriangle(vertices[10], vertices[0], vertices[1], vertices[2]));
  EXPECT_FALSE(pathfinder::math::isPointOnTriangle(vertices[11], vertices[0], vertices[1], vertices[2]));
  EXPECT_FALSE(pathfinder::math::isPointOnTriangle(vertices[12], vertices[0], vertices[1], vertices[2]));

  // Point is on an edge of the triangle
  EXPECT_TRUE(pathfinder::math::isPointOnTriangle(vertices[3], vertices[0], vertices[1], vertices[2]));
  EXPECT_TRUE(pathfinder::math::isPointOnTriangle(vertices[4], vertices[0], vertices[1], vertices[2]));
  EXPECT_TRUE(pathfinder::math::isPointOnTriangle(vertices[5], vertices[0], vertices[1], vertices[2]));

  // Point is on a vertex of the triangle
  EXPECT_TRUE(pathfinder::math::isPointOnTriangle(vertices[0], vertices[0], vertices[1], vertices[2]));
  EXPECT_TRUE(pathfinder::math::isPointOnTriangle(vertices[1], vertices[0], vertices[1], vertices[2]));
  EXPECT_TRUE(pathfinder::math::isPointOnTriangle(vertices[2], vertices[0], vertices[1], vertices[2]));
}

TEST(Pathfinder_Math, IsPointOnLineSegment) {
  const std::vector<pathfinder::Vector> vertices{{  1.0,  1.0}, // 0
                                                 {  3.0,  1.0}, // 1
                                                 {  2.0,  1.0}, // 2
                                                 {  0.0,  1.0}, // 3
                                                 {  4.0,  1.0}, // 4
                                                 {  1.0, 0.0}, // 5
                                                 {  2.0, 0.0}, // 6
                                                 {  3.0, 0.0}, // 7
                                                 {  1.0, 2.0}, // 8
                                                 {  2.0, 2.0}, // 9
                                                 {  3.0, 2.0}, // 10
                                                 {  2.0, 1.0}, // 11
                                                 {  6.0, 3.0}, // 12
                                                 {  4.0, 2.0}, // 13
                                                 {  0.0, 0.0}, // 14
                                                 {  8.0, 4.0}, // 15
                                                 {  4.0, 1.0}, // 16
                                                 {  4.0, 3.0}, // 17
                                                };
  // Horizontal line
  // Point is on the segment
  EXPECT_TRUE(pathfinder::math::isPointOnLineSegment(vertices[0], vertices[0], vertices[1]));
  EXPECT_TRUE(pathfinder::math::isPointOnLineSegment(vertices[1], vertices[0], vertices[1]));
  EXPECT_TRUE(pathfinder::math::isPointOnLineSegment(vertices[2], vertices[0], vertices[1]));

  // Point is not on the segment
  EXPECT_FALSE(pathfinder::math::isPointOnLineSegment(vertices[3], vertices[0], vertices[1]));
  EXPECT_FALSE(pathfinder::math::isPointOnLineSegment(vertices[4], vertices[0], vertices[1]));
  EXPECT_FALSE(pathfinder::math::isPointOnLineSegment(vertices[5], vertices[0], vertices[1]));
  EXPECT_FALSE(pathfinder::math::isPointOnLineSegment(vertices[6], vertices[0], vertices[1]));
  EXPECT_FALSE(pathfinder::math::isPointOnLineSegment(vertices[7], vertices[0], vertices[1]));
  EXPECT_FALSE(pathfinder::math::isPointOnLineSegment(vertices[8], vertices[0], vertices[1]));
  EXPECT_FALSE(pathfinder::math::isPointOnLineSegment(vertices[9], vertices[0], vertices[1]));
  EXPECT_FALSE(pathfinder::math::isPointOnLineSegment(vertices[10], vertices[0], vertices[1]));

  // Diagonal line
  // Point is on the segment
  EXPECT_TRUE(pathfinder::math::isPointOnLineSegment(vertices[11], vertices[11], vertices[12]));
  EXPECT_TRUE(pathfinder::math::isPointOnLineSegment(vertices[12], vertices[11], vertices[12]));
  EXPECT_TRUE(pathfinder::math::isPointOnLineSegment(vertices[13], vertices[11], vertices[12]));

  // Point is not on the segment
  EXPECT_FALSE(pathfinder::math::isPointOnLineSegment(vertices[14], vertices[11], vertices[12]));
  EXPECT_FALSE(pathfinder::math::isPointOnLineSegment(vertices[15], vertices[11], vertices[12]));
  EXPECT_FALSE(pathfinder::math::isPointOnLineSegment(vertices[16], vertices[11], vertices[12]));
  EXPECT_FALSE(pathfinder::math::isPointOnLineSegment(vertices[17], vertices[11], vertices[12]));
}

TEST(Pathfinder_Math, TrianglesOverlap) {
  // bool trianglesOverlap(const Vector &triangle1V1, const Vector &triangle1V2, const Vector &triangle1V3, const Vector &triangle2V1, const Vector &triangle2V2, const Vector &triangle2V3);
  const std::vector<pathfinder::Vector> vertices{{  0.0,  0.0}, // 0
                                                 {  5.0,  0.0}, // 1
                                                 {  0.0,  5.0}, // 2
                                                 { -5.0,  0.0}, // 3
                                                 {  0.0, -5.0}, // 4
                                                 {  5.0,  5.0}, // 5
                                                 { -5.0,  5.0}, // 6
                                                 {  5.0, -5.0}, // 7
                                                 { -4.0,  1.0}, // 8
                                                 { -4.0,  0.0}, // 9
                                                 {  1.0, -4.0}, // 10
                                                 {  0.0, -4.0}, // 11
                                                 {  5.0,  4.0}, // 12
                                                 {  4.0,  5.0}, // 13
                                                 {  2.0,  3.0}, // 14
                                                 {  3.0,  2.0}, // 15
                                                 {  3.0,  3.0}, // 16
                                                 {  2.0,  2.0}, // 17
                                                 {  2.0,  1.0}, // 18
                                                 {  1.0,  1.0}, // 19
                                                };

  // Triangle overlaps with self
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[0], vertices[1], vertices[2], true));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[0], vertices[1], vertices[2], false));

  // Sharing only a vertex
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[0], vertices[3], vertices[4], true));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[0], vertices[3], vertices[4], false));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[1], vertices[4], vertices[7], true));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[1], vertices[4], vertices[7], false));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[2], vertices[3], vertices[6], true));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[2], vertices[3], vertices[6], false));

  // Completely sharing an edge
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[0], vertices[2], vertices[3], true));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[0], vertices[2], vertices[3], false));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[0], vertices[4], vertices[1], true));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[0], vertices[4], vertices[1], false));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[1], vertices[5], vertices[2], true));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[1], vertices[5], vertices[2], false));

  // Off to the side, not touching
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[9], vertices[8], vertices[3], true));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[9], vertices[8], vertices[3], false));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[11], vertices[4], vertices[10], true));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[11], vertices[4], vertices[10], false));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[12], vertices[5], vertices[13], true));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[12], vertices[5], vertices[13], false));

  // Sharing a partial edge
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[15], vertices[16], vertices[14], true));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[15], vertices[16], vertices[14], false));

  // Sharing vertex on edge
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[15], vertices[12], vertices[16], true));
  EXPECT_FALSE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[15], vertices[12], vertices[16], false));

  // Small triangle inside, sharing an edge
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[15], vertices[17], vertices[14], true));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[15], vertices[17], vertices[14], false));

  // Small triangle inside, sharing vertex on edge
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[15], vertices[18], vertices[17], true));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[15], vertices[18], vertices[17], false));

  // Small triangle inside
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[19], vertices[18], vertices[17], true));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[19], vertices[18], vertices[17], false));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[19], vertices[18], vertices[17], vertices[0], vertices[1], vertices[2], true));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[19], vertices[18], vertices[17], vertices[0], vertices[1], vertices[2], false));

  // Messy overlapping (non-zero area)
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[18], vertices[7], vertices[10], true));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[18], vertices[7], vertices[10], false));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[16], vertices[3], vertices[4], true));
  EXPECT_TRUE(pathfinder::math::trianglesOverlap(vertices[0], vertices[1], vertices[2], vertices[16], vertices[3], vertices[4], false));

  // Different triangle winding orders
  {
    std::array<std::tuple<pathfinder::Vector, pathfinder::Vector, pathfinder::Vector>, 6> t1 = {{{vertices[0], vertices[1], vertices[2]},
                                                                                                {vertices[0], vertices[2], vertices[1]},
                                                                                                {vertices[1], vertices[0], vertices[2]},
                                                                                                {vertices[1], vertices[2], vertices[0]},
                                                                                                {vertices[2], vertices[0], vertices[1]},
                                                                                                {vertices[2], vertices[1], vertices[0]}}};
    std::array<std::tuple<pathfinder::Vector, pathfinder::Vector, pathfinder::Vector>, 6> t2 = {{{vertices[19], vertices[18], vertices[17]},
                                                                                                {vertices[19], vertices[17], vertices[18]},
                                                                                                {vertices[18], vertices[19], vertices[17]},
                                                                                                {vertices[18], vertices[17], vertices[19]},
                                                                                                {vertices[17], vertices[19], vertices[18]},
                                                                                                {vertices[17], vertices[18], vertices[19]}}};
    // Different triangles, all possible winding orders
    for (const auto &i : t1) {
      for (const auto &j : t2) {
        EXPECT_TRUE(pathfinder::math::trianglesOverlap(std::get<0>(i), std::get<1>(i), std::get<2>(i), std::get<0>(j), std::get<1>(j), std::get<2>(j), true));
        EXPECT_TRUE(pathfinder::math::trianglesOverlap(std::get<0>(i), std::get<1>(i), std::get<2>(i), std::get<0>(j), std::get<1>(j), std::get<2>(j), false));
      }
    }
    // Same triangle against itself, all possible winding orders
    for (const auto &i : t1) {
      for (const auto &j : t1) {
        EXPECT_TRUE(pathfinder::math::trianglesOverlap(std::get<0>(i), std::get<1>(i), std::get<2>(i), std::get<0>(j), std::get<1>(j), std::get<2>(j), true));
        EXPECT_TRUE(pathfinder::math::trianglesOverlap(std::get<0>(i), std::get<1>(i), std::get<2>(i), std::get<0>(j), std::get<1>(j), std::get<2>(j), false));
      }
    }
  }
}