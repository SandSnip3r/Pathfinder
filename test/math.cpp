#include "../math_helpers.h"

#include <gtest/gtest.h>

#include <absl/log/log.h>
#include <absl/strings/str_format.h>

#include <array>
#include <utility>

#define ASSERT_IN_RANGE(VAL, MIN, MAX) \
  ASSERT_GE((VAL), (MIN));             \
  ASSERT_LE((VAL), (MAX))

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

TEST(Pathfinder_Math, AngleTwoVectors) {
  std::vector<std::tuple<pathfinder::Vector, pathfinder::Vector, double>> testCases = {
    {{0.0, 0.0}, { 1.0,  0.0}, 0.0},
    {{0.0, 0.0}, { 1.0,  1.0},   pathfinder::math::kPi/4.0},
    {{0.0, 0.0}, { 0.0,  1.0},   pathfinder::math::kPi/2.0},
    {{0.0, 0.0}, {-1.0,  1.0}, 3*pathfinder::math::kPi/4.0},
    {{0.0, 0.0}, {-1.0,  0.0},   pathfinder::math::kPi},
    {{0.0, 0.0}, {-1.0, -1.0}, 5*pathfinder::math::kPi/4.0},
    {{0.0, 0.0}, { 0.0, -1.0}, 3*pathfinder::math::kPi/2.0},
    {{0.0, 0.0}, { 1.0, -1.0}, 7*pathfinder::math::kPi/4.0}
  };
  std::vector<double> scalings = {
    0.1, 0.01, 0.001, 0.0001, 0.00001,
    10.0, 100.0, 1000.0, 10'000.0, 100'000.0,
    2.0, 0.16833546, 65471.35984,
  };
  std::vector<double> translations = {
    1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001, 2.0, 10.0, 100.0, 123.456, 100000.0
  };
  for (const auto &testCase : testCases) {
    const pathfinder::Vector &start = std::get<0>(testCase);
    const pathfinder::Vector &end = std::get<1>(testCase);
    const double expectedAngle = std::get<2>(testCase);
    // Test the case as it is.
    {
      const double angle = pathfinder::math::angle(start, end);
      EXPECT_EQ(expectedAngle, angle);
    }
    // Test the case when scaled.
    for (const double scale : scalings) {
      const pathfinder::Vector scaledStart(start.x()*scale, start.y()*scale);
      const pathfinder::Vector scaledEnd(end.x()*scale, end.y()*scale);
      const double angle = pathfinder::math::angle(scaledStart, scaledEnd);
      EXPECT_EQ(expectedAngle, angle);
    }
    // Test the case when translated.
    for (const double translation : translations) {
      for (const double signedTranslation : { +translation, -translation }) {
        const pathfinder::Vector translatedStart(start.x()+signedTranslation, start.y()+signedTranslation);
        const pathfinder::Vector translatedEnd(end.x()+signedTranslation, end.y()+signedTranslation);
        const double angle = pathfinder::math::angle(translatedStart, translatedEnd);
        EXPECT_EQ(expectedAngle, angle);
      }
    }
    // Test the case when scaled and translated.
    for (const double scale : scalings) {
      for (const double translation : translations) {
        for (const double signedTranslation : { +translation, -translation }) {
          const pathfinder::Vector transformedStart(start.x()*scale + signedTranslation, start.y()*scale + signedTranslation);
          const pathfinder::Vector transformedEnd(end.x()*scale + signedTranslation, end.y()*scale + signedTranslation);
          const double angle = pathfinder::math::angle(transformedStart, transformedEnd);
          EXPECT_NEAR(expectedAngle, angle, 1e-11); // 1e-12 fails
        }
      }
    }
  }
  {
    const pathfinder::Vector start{500.0, 458.33333333333337122895};
    const pathfinder::Vector end{500.0, 750.0};
    const double angle = pathfinder::math::angle(start, end);
    EXPECT_NEAR(angle, pathfinder::math::kPi/2.0, 1e-20);
  }
  {
    const pathfinder::Vector start{500.00000000000005684342,458.33333333333337122895};
    const pathfinder::Vector end{500.0, 750.0};
    const double angle = pathfinder::math::angle(start, end);
    EXPECT_NEAR(angle, pathfinder::math::kPi/2.0, 1e-15);
  }
  {
    const pathfinder::Vector start{500.001,458.33333333333337122895};
    const pathfinder::Vector end{500.0, 750.0};
    const double angle = pathfinder::math::angle(start, end);
    EXPECT_NEAR(angle, pathfinder::math::kPi/2.0, 1e-5);
  }
  {
    const pathfinder::Vector start{458.33333333333337122895,500.00000000000005684342};
    const pathfinder::Vector end{750.0, 500.0};
    const double angle = pathfinder::math::angle(start, end);
    EXPECT_LE(angle, pathfinder::math::k2Pi);
  }
  {
    const pathfinder::Vector start{458.33333333333337122895,500.0000001};
    const pathfinder::Vector end{750.0, 500.0};
    const double angle = pathfinder::math::angle(start, end);
    EXPECT_LE(angle, pathfinder::math::k2Pi);
  }
}

TEST(Pathfinder_Math, IntersectionsPointsOfTangentLinesToCircle) {
  {
    const pathfinder::Vector point{500.0,458.33333333333337122895};
    const pathfinder::Vector centerOfCircle{500.0, 750.0};
    const double radius{10.0};
    const auto [firstIntersection,secondIntersection] = pathfinder::math::intersectionsPointsOfTangentLinesToCircle(point, centerOfCircle, radius);
    EXPECT_NEAR(firstIntersection.x(), 490.0058792793, 1e-9);
    EXPECT_NEAR(firstIntersection.y(), 749.6571428571, 1e-9);
    EXPECT_NEAR(secondIntersection.x(), 509.9941207207, 1e-9);
    EXPECT_NEAR(secondIntersection.y(), 749.6571428571, 1e-9);
  }
  {
    const pathfinder::Vector point{500.00000000000005684342,458.33333333333337122895};
    const pathfinder::Vector centerOfCircle{500.0, 750.0};
    const double radius{10.0};
    const auto [firstIntersection,secondIntersection] = pathfinder::math::intersectionsPointsOfTangentLinesToCircle(point, centerOfCircle, radius);
    EXPECT_NEAR(firstIntersection.x(), 490.0058792793, 1e-9);
    EXPECT_NEAR(firstIntersection.y(), 749.6571428571, 1e-9);
    EXPECT_NEAR(secondIntersection.x(), 509.9941207207, 1e-9);
    EXPECT_NEAR(secondIntersection.y(), 749.6571428571, 1e-9);
  }

  // TODO: Test more
}

using CircleConsciousParams = std::tuple<pathfinder::Vector, pathfinder::Vector, pathfinder::AngleDirection, pathfinder::AngleDirection, pathfinder::Vector, pathfinder::Vector>;
class ParamCircleConsciousLine : public testing::TestWithParam<CircleConsciousParams> {};

TEST_P(ParamCircleConsciousLine, TestName) {
  constexpr double kRadius{1.0};
  constexpr double kAbsErrorTolerance{8e-14};
  const auto &param = GetParam();
  const auto [lineStart, lineEnd] = pathfinder::math::createCircleConsciousLine(std::get<0>(param), std::get<2>(param), std::get<1>(param), std::get<3>(param), kRadius);
  EXPECT_NEAR(lineStart.x(), std::get<4>(param).x(), kAbsErrorTolerance);
  EXPECT_NEAR(lineStart.y(), std::get<4>(param).y(), kAbsErrorTolerance);
  EXPECT_NEAR(lineEnd.x(), std::get<5>(param).x(), kAbsErrorTolerance);
  EXPECT_NEAR(lineEnd.y(), std::get<5>(param).y(), kAbsErrorTolerance);
}

INSTANTIATE_TEST_SUITE_P(TestSuite,
                         ParamCircleConsciousLine,
                         testing::Values(
    CircleConsciousParams{{ 0.0,0.0}, {10.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kClockwise,        { 0.0, 1.0},             {10.0, 1.0}},
    CircleConsciousParams{{ 0.0,0.0}, {10.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kCounterclockwise, { 0.0,-1.0},             {10.0,-1.0}},
    CircleConsciousParams{{ 0.0,0.0}, {10.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kCounterclockwise, { 0.2, 0.9797958971132}, { 9.8,-0.9797958971133}},
    CircleConsciousParams{{ 0.0,0.0}, {10.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kClockwise,        { 0.2,-0.9797958971132}, { 9.8, 0.9797958971133}},
    CircleConsciousParams{{ 0.0,0.0}, {10.0,0.0}, pathfinder::AngleDirection::kNoDirection,      pathfinder::AngleDirection::kClockwise,        { 0.0, 0.0},             { 9.9, 0.9949874371066}},
    CircleConsciousParams{{ 0.0,0.0}, {10.0,0.0}, pathfinder::AngleDirection::kNoDirection,      pathfinder::AngleDirection::kCounterclockwise, { 0.0, 0.0},             { 9.9,-0.9949874371066}},
    CircleConsciousParams{{ 0.0,0.0}, {10.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kNoDirection,      { 0.1,-0.9949874371066}, {10.0, 0.0}},
    CircleConsciousParams{{ 0.0,0.0}, {10.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kNoDirection,      { 0.1, 0.9949874371066}, {10.0, 0.0}},
    CircleConsciousParams{{10.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kClockwise,        {10.0,-1.0},             { 0.0,-1.0}},
    CircleConsciousParams{{10.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kCounterclockwise, {10.0, 1.0},             { 0.0, 1.0}},
    CircleConsciousParams{{10.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kCounterclockwise, { 9.8,-0.9797958971133}, { 0.2, 0.9797958971132}},
    CircleConsciousParams{{10.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kClockwise,        { 9.8, 0.9797958971133}, { 0.2,-0.9797958971132}},
    CircleConsciousParams{{10.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kNoDirection,      pathfinder::AngleDirection::kClockwise,        {10.0, 0.0},             { 0.1,-0.9949874371066}},
    CircleConsciousParams{{10.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kNoDirection,      pathfinder::AngleDirection::kCounterclockwise, {10.0, 0.0},             { 0.1, 0.9949874371066}},
    CircleConsciousParams{{10.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kNoDirection,      { 9.9, 0.9949874371066}, { 0.0, 0.0}},
    CircleConsciousParams{{10.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kNoDirection,      { 9.9,-0.9949874371066}, { 0.0, 0.0}},

    CircleConsciousParams{{ 0.0,0.0}, { 2.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kCounterclockwise, { 1.0, 0.0},             { 1.0, 0.0}},
    CircleConsciousParams{{ 0.0,0.0}, { 2.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kClockwise,        { 1.0, 0.0},             { 1.0, 0.0}},
    CircleConsciousParams{{ 2.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kCounterclockwise, { 1.0, 0.0},             { 1.0, 0.0}},
    CircleConsciousParams{{ 2.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kClockwise,        { 1.0, 0.0},             { 1.0, 0.0}},
    CircleConsciousParams{{ 0.0,0.0}, { 2.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kClockwise,        { 0.0, 1.0},             { 2.0, 1.0}},
    CircleConsciousParams{{ 0.0,0.0}, { 2.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kCounterclockwise, { 0.0,-1.0},             { 2.0,-1.0}},
    CircleConsciousParams{{ 2.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kClockwise,        { 2.0,-1.0},             { 0.0,-1.0}},
    CircleConsciousParams{{ 2.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kCounterclockwise, { 2.0, 1.0},             { 0.0, 1.0}},

    CircleConsciousParams{{ 0.0,0.0}, { 1.0,0.0}, pathfinder::AngleDirection::kNoDirection,      pathfinder::AngleDirection::kClockwise,        { 0.0, 0.0},             { 0.0, 0.0}},
    CircleConsciousParams{{ 0.0,0.0}, { 1.0,0.0}, pathfinder::AngleDirection::kNoDirection,      pathfinder::AngleDirection::kCounterclockwise, { 0.0, 0.0},             { 0.0, 0.0}},
    CircleConsciousParams{{ 0.0,0.0}, { 1.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kNoDirection,      { 1.0, 0.0},             { 1.0, 0.0}},
    CircleConsciousParams{{ 0.0,0.0}, { 1.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kNoDirection,      { 1.0, 0.0},             { 1.0, 0.0}},
    CircleConsciousParams{{ 1.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kNoDirection,      pathfinder::AngleDirection::kClockwise,        { 1.0, 0.0},             { 1.0, 0.0}},
    CircleConsciousParams{{ 1.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kNoDirection,      pathfinder::AngleDirection::kCounterclockwise, { 1.0, 0.0},             { 1.0, 0.0}},
    CircleConsciousParams{{ 1.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kNoDirection,      { 0.0, 0.0},             { 0.0, 0.0}},
    CircleConsciousParams{{ 1.0,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kNoDirection,      { 0.0, 0.0},             { 0.0, 0.0}},

    CircleConsciousParams{{ 0.0,0.0}, { 0.5,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kClockwise,        { 0.0, 1.0},             { 0.5, 1.0}},
    CircleConsciousParams{{ 0.0,0.0}, { 0.5,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kCounterclockwise, { 0.0,-1.0},             { 0.5,-1.0}},
    CircleConsciousParams{{ 0.5,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kClockwise,        { 0.5,-1.0},             { 0.0,-1.0}},
    CircleConsciousParams{{ 0.5,0.0}, { 0.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kCounterclockwise, { 0.5, 1.0},             { 0.0, 1.0}},

    CircleConsciousParams{{ 0.0,0.0}, { 0.0,0.5}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kClockwise,        {-1.0, 0.0},             {-1.0, 0.5}},
    CircleConsciousParams{{ 0.0,0.0}, { 0.0,0.5}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kCounterclockwise, { 1.0, 0.0},             { 1.0, 0.5}},
    CircleConsciousParams{{ 0.0,0.5}, { 0.0,0.0}, pathfinder::AngleDirection::kClockwise,        pathfinder::AngleDirection::kClockwise,        { 1.0, 0.5},             { 1.0, 0.0}},
    CircleConsciousParams{{ 0.0,0.5}, { 0.0,0.0}, pathfinder::AngleDirection::kCounterclockwise, pathfinder::AngleDirection::kCounterclockwise, {-1.0, 0.5},             {-1.0, 0.0}}
));

TEST(Pathfinder_Math, CreateVectorTangentToPointOnCircle) {
  const pathfinder::Vector center{10.0, 10.0};
  const double radius{1.0};
  const double kSqrt2Over2{sqrt(2)/2.0};
  const std::vector<pathfinder::Vector> pointsOnCircumferences {
    {center.x() +  1.0,         center.y() +  0.0},
    {center.x() +  0.0,         center.y() +  1.0},
    {center.x() + -1.0,         center.y() +  0.0},
    {center.x() +  0.0,         center.y() + -1.0},
    {center.x() +  kSqrt2Over2, center.y() +  kSqrt2Over2},
    {center.x() + -kSqrt2Over2, center.y() +  kSqrt2Over2},
    {center.x() + -kSqrt2Over2, center.y() + -kSqrt2Over2},
    {center.x() +  kSqrt2Over2, center.y() + -kSqrt2Over2},
  };
  for (const auto &poc : pointsOnCircumferences) {
    const auto [p1, p2] = pathfinder::math::createVectorTangentToPointOnCircle(center, radius, poc);
    LOG(INFO) << absl::StreamFormat("Point in circumference (%.10f,%.10f) results in vector Segment((%.10f,%.10f),(%.10f,%.10f))", poc.x(), poc.y(), p1.x(), p1.y(), p2.x(), p2.y());

    // 6: I0121 18:44:10.863611     627 math.cpp:216] Point in circumference (11.0000000000,10.0000000000) results in vector Segment((11.0000000000,9.5000000000),(11.0000000000,10.5000000000))
    // 6: I0121 18:44:10.863638     627 math.cpp:216] Point in circumference (10.0000000000,11.0000000000) results in vector Segment((-1.0000000000,11.0000000000),(1.0000000000,11.0000000000))
    // 6: I0121 18:44:10.863658     627 math.cpp:216] Point in circumference (9.0000000000,10.0000000000) results in vector Segment((9.0000000000,9.5000000000),(9.0000000000,10.5000000000))
    // 6: I0121 18:44:10.863679     627 math.cpp:216] Point in circumference (10.0000000000,9.0000000000) results in vector Segment((-1.0000000000,9.0000000000),(1.0000000000,9.0000000000))
    // 6: I0121 18:44:10.863700     627 math.cpp:216] Point in circumference (10.7071067812,10.7071067812) results in vector Segment((-1.0000000000,22.4142135624),(1.0000000000,20.4142135624))
    // 6: I0121 18:44:10.863722     627 math.cpp:216] Point in circumference (9.2928932188,10.7071067812) results in vector Segment((-1.0000000000,0.4142135624),(1.0000000000,2.4142135624))
    // 6: I0121 18:44:10.863745     627 math.cpp:216] Point in circumference (9.2928932188,9.2928932188) results in vector Segment((-1.0000000000,19.5857864376),(1.0000000000,17.5857864376))
    // 6: I0121 18:44:10.863767     627 math.cpp:216] Point in circumference (10.7071067812,9.2928932188) results in vector Segment((-1.0000000000,-2.4142135624),(1.0000000000,-0.4142135624))
  }
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

TEST(Pathfinder_Math, CreatePerpendicularBisector) {
  const std::vector<pathfinder::Vector> vertices{{ 0.0,  0.0}, // 0
                                                 { 1.0,  0.0}, // 1
                                                 { 7.0,  0.0}, // 2
                                                 { 1.0,  1.0}, // 3
                                                 { 2.0,  2.0}, // 4
                                                 { 0.0,  1.0}, // 5
                                                 { 0.0,  7.0}, // 6
                                                 {-1.0,  1.0}, // 7
                                                 {-2.0,  2.0}, // 8
                                                 {-1.0,  0.0}, // 9
                                                 {-7.0,  0.0}, // 10
                                                 {-1.0, -1.0}, // 11
                                                 {-2.0, -2.0}, // 12
                                                 { 0.0, -1.0}, // 13
                                                 { 0.0, -7.0}, // 14
                                                 { 1.0, -1.0}, // 15
                                                 { 2.0, -2.0}  // 16
                                                };
  std::vector<std::tuple<size_t,size_t,size_t,size_t>> values {
    // Horizontal inputs
    {10,2,14,6},
    {9,1,13,5},
    // Vertical inputs
    {14,6,2,10},
    {13,5,1,9},
    // Positive slope diagonals
    {12,4,16,8},
    {11,3,15,7},
    // Negative slope diagonals
    {8,16,12,4},
    {7,15,11,3},
  };
  for (const auto &value : values) {
    const auto &inStart = vertices.at(std::get<0>(value));
    const auto &inEnd = vertices.at(std::get<1>(value));
    const auto &outStart = vertices.at(std::get<2>(value));
    const auto &outEnd = vertices.at(std::get<3>(value));
    const auto distanceBetween = pathfinder::math::distance(inStart, inEnd);
    {
      // A->B
      const auto [lineStart, lineEnd] = pathfinder::math::createPerpendicularBisector(inStart, inEnd, distanceBetween);
      EXPECT_EQ(lineStart.x(), outStart.x());
      EXPECT_EQ(lineStart.y(), outStart.y());
      EXPECT_EQ(lineEnd.x(), outEnd.x());
      EXPECT_EQ(lineEnd.y(), outEnd.y());
    }
    // A reflected input should produce a reflected output.
    {
      // B->A
      const auto [lineStart, lineEnd] = pathfinder::math::createPerpendicularBisector(inEnd, inStart, distanceBetween);
      EXPECT_EQ(lineStart.x(), outEnd.x());
      EXPECT_EQ(lineStart.y(), outEnd.y());
      EXPECT_EQ(lineEnd.x(), outStart.x());
      EXPECT_EQ(lineEnd.y(), outStart.y());
    }
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

TEST(Pathfinder_Math, IntersectForIntervals) {
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
  // Plus-shaped intersection
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[1], vertices[2], vertices[3], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.5);
    ASSERT_EQ(i2, 0.5);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[1], vertices[3], vertices[2], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.5);
    ASSERT_EQ(i2, 0.5);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[1], vertices[0], vertices[2], vertices[3], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.5);
    ASSERT_EQ(i2, 0.5);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[1], vertices[0], vertices[3], vertices[2], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.5);
    ASSERT_EQ(i2, 0.5);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[0], vertices[1], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.5);
    ASSERT_EQ(i2, 0.5);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[3], vertices[2], vertices[0], vertices[1], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.5);
    ASSERT_EQ(i2, 0.5);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[1], vertices[0], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.5);
    ASSERT_EQ(i2, 0.5);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[3], vertices[2], vertices[1], vertices[0], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.5);
    ASSERT_EQ(i2, 0.5);
  }

  {
    // Intersection of self, horizonal
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[2], vertices[3]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    // Intersection of self, vertical
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[1], vertices[0], vertices[1]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    // Intersection of self, diagonal
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[3], vertices[0], vertices[3]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }

  // A short diagonal and long horizontal. Intersection only on the long horizontal.
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[4], vertices[5], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1,  0.2);
    ASSERT_EQ(i2, -1.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[5], vertices[4], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, 0.2);
    ASSERT_EQ(i2, 2.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[3], vertices[2], vertices[4], vertices[5], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1,  0.8);
    ASSERT_EQ(i2, -1.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[3], vertices[2], vertices[5], vertices[4], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, 0.8);
    ASSERT_EQ(i2, 2.0);
  }

  // A short diagonal and long horizontal. No intersection on either segment.
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[6], vertices[7], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, -0.3);
    ASSERT_EQ(i2, -1.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[7], vertices[6], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, -0.3);
    ASSERT_EQ(i2,  2.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[3], vertices[2], vertices[6], vertices[7], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1,  1.3);
    ASSERT_EQ(i2, -1.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[3], vertices[2], vertices[7], vertices[6], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, 1.3);
    ASSERT_EQ(i2, 2.0);
  }

  // A short horizontal above a long horizontal (within x interval), parallel to each other. No intersection ever.
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[8], vertices[5], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[5], vertices[8], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[3], vertices[2], vertices[8], vertices[5], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[3], vertices[2], vertices[5], vertices[8], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[8], vertices[5], vertices[2], vertices[3], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[5], vertices[8], vertices[2], vertices[3], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[8], vertices[5], vertices[3], vertices[2], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[5], vertices[8], vertices[3], vertices[2], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }

  // A short horizontal above a long horizontal (NOT within x interval), parallel to each other. No intersection ever.
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[9], vertices[7], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[7], vertices[9], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[3], vertices[2], vertices[9], vertices[7], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[3], vertices[2], vertices[7], vertices[9], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[9], vertices[7], vertices[2], vertices[3], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[7], vertices[9], vertices[2], vertices[3], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[9], vertices[7], vertices[3], vertices[2], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[7], vertices[9], vertices[3], vertices[2], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }

  // A horizontal above another horizontal (partially within x interval), parallel to each other. No intersection ever.
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[9], vertices[5], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[2], vertices[3], vertices[5], vertices[9], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[3], vertices[2], vertices[9], vertices[5], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[3], vertices[2], vertices[5], vertices[9], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[9], vertices[5], vertices[2], vertices[3], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[5], vertices[9], vertices[2], vertices[3], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[9], vertices[5], vertices[3], vertices[2], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[5], vertices[9], vertices[3], vertices[2], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
    ASSERT_EQ(i1, std::numeric_limits<double>::infinity());
    ASSERT_EQ(i2, std::numeric_limits<double>::infinity());
  }

  // Two line segments which share endpoints.
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[1], vertices[0], vertices[0], vertices[10], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 1.0);
    ASSERT_EQ(i2, 0.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[1], vertices[0], vertices[10], vertices[0], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 1.0);
    ASSERT_EQ(i2, 1.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[1], vertices[0], vertices[0], vertices[11], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 1.0);
    ASSERT_EQ(i2, 0.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[1], vertices[0], vertices[0], vertices[12], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 1.0);
    ASSERT_EQ(i2, 0.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[1], vertices[0], vertices[10], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.0);
    ASSERT_EQ(i2, 0.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[1], vertices[10], vertices[0], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.0);
    ASSERT_EQ(i2, 1.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[1], vertices[0], vertices[11], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.0);
    ASSERT_EQ(i2, 0.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[1], vertices[0], vertices[12], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.0);
    ASSERT_EQ(i2, 0.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[10], vertices[1], vertices[0], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.0);
    ASSERT_EQ(i2, 1.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[10], vertices[0], vertices[1], vertices[0], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 1.0);
    ASSERT_EQ(i2, 1.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[11], vertices[1], vertices[0], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.0);
    ASSERT_EQ(i2, 1.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[12], vertices[1], vertices[0], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.0);
    ASSERT_EQ(i2, 1.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[10], vertices[0], vertices[1], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.0);
    ASSERT_EQ(i2, 0.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[10], vertices[0], vertices[0], vertices[1], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 1.0);
    ASSERT_EQ(i2, 0.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[11], vertices[0], vertices[1], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.0);
    ASSERT_EQ(i2, 0.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[12], vertices[0], vertices[1], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.0);
    ASSERT_EQ(i2, 0.0);
  }

  // Two line segments which interlap for some distance resulting in infinite overlap.
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[1], vertices[0], vertices[0], vertices[13]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[1], vertices[0], vertices[13], vertices[0]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[1], vertices[0], vertices[13]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[1], vertices[13], vertices[0]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[13], vertices[1], vertices[0]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[13], vertices[0], vertices[1], vertices[0]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[13], vertices[0], vertices[1]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[13], vertices[0], vertices[0], vertices[1]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kInfinite);
  }

  // Colinear but segments are not touching.
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[9], vertices[7], vertices[8], vertices[5]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[9], vertices[7], vertices[5], vertices[8]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[7], vertices[9], vertices[8], vertices[5]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[7], vertices[9], vertices[5], vertices[8]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[8], vertices[5], vertices[9], vertices[7]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[5], vertices[8], vertices[9], vertices[7]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[8], vertices[5], vertices[7], vertices[9]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[5], vertices[8], vertices[7], vertices[9]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }

  // A 0-length line segment on another line segment.
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[1], vertices[13], vertices[13], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.1);
    ASSERT_IN_RANGE(i2, 0.0, 1.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[1], vertices[0], vertices[13], vertices[13], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_EQ(i1, 0.9);
    ASSERT_IN_RANGE(i2, 0.0, 1.0);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[13], vertices[13], vertices[0], vertices[1], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_IN_RANGE(i1, 0.0, 1.0);
    ASSERT_EQ(i2, 0.1);
  }
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[13], vertices[13], vertices[1], vertices[0], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_IN_RANGE(i1, 0.0, 1.0);
    ASSERT_EQ(i2, 0.9);
  }

  // A 0-length line that is not on the other line but is colinear with it.
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[10], vertices[13], vertices[1], vertices[1]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[13], vertices[10], vertices[1], vertices[1]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[1], vertices[1], vertices[10], vertices[13]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[1], vertices[1], vertices[13], vertices[10]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }

  // A 0-length line that is not on nor colinear with the other line.
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[10], vertices[13], vertices[11], vertices[11]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[13], vertices[10], vertices[11], vertices[11]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[11], vertices[11], vertices[10], vertices[13]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[11], vertices[11], vertices[13], vertices[10]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }

  // Both lines are 0-length and not the same.
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[0], vertices[1], vertices[1]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }
  {
    const auto result = pathfinder::math::intersectForIntervals(vertices[1], vertices[1], vertices[0], vertices[0]);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kNone);
  }

  // All 4 points are the same.
  {
    double i1, i2;
    const auto result = pathfinder::math::intersectForIntervals(vertices[0], vertices[0], vertices[0], vertices[0], &i1, &i2);
    ASSERT_EQ(result, pathfinder::math::IntersectionResult::kOne);
    ASSERT_IN_RANGE(i1, 0.0, 1.0);
    ASSERT_IN_RANGE(i2, 0.0, 1.0);
  }
}

TEST(Pathfinder_Math, ReflectPointOverLine) {
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
                                                 { 0.0, -4.0}, // 13
                                                 {-1.0, -5.0}  // 14
                                                };

  // The point reflects over the middle of the line.
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[2], vertices[0], vertices[1]);
    ASSERT_EQ(vertices[3], reflectedPoint);
  }
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[2], vertices[1], vertices[0]);
    ASSERT_EQ(vertices[3], reflectedPoint);
  }

  // The point is colinear with the line and not on the line.
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[10], vertices[0], vertices[1]);
    ASSERT_EQ(vertices[10], reflectedPoint);
  }
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[10], vertices[1], vertices[0]);
    ASSERT_EQ(vertices[10], reflectedPoint);
  }

  // The point is on the line.
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[13], vertices[0], vertices[1]);
    ASSERT_EQ(vertices[13], reflectedPoint);
  }
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[13], vertices[1], vertices[0]);
    ASSERT_EQ(vertices[13], reflectedPoint);
  }

  // The point reflects over the endpoint of the line.
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[11], vertices[0], vertices[1]);
    ASSERT_EQ(vertices[14], reflectedPoint);
  }
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[11], vertices[1], vertices[0]);
    ASSERT_EQ(vertices[14], reflectedPoint);
  }

  // The point reflects over a point beyond the extent of the line.
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[11], vertices[13], vertices[1]);
    ASSERT_EQ(vertices[14], reflectedPoint);
  }
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[11], vertices[1], vertices[13]);
    ASSERT_EQ(vertices[14], reflectedPoint);
  }

  // The point reflects over a point within a diagonal line.
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[13], vertices[0], vertices[12]);
    ASSERT_EQ(vertices[11], reflectedPoint);
  }
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[13], vertices[12], vertices[0]);
    ASSERT_EQ(vertices[11], reflectedPoint);
  }

  // The point reflects over a point beyond the extent of a diagonal line.
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[14], vertices[0], vertices[12]);
    ASSERT_EQ(vertices[10], reflectedPoint);
  }
  {
    const auto reflectedPoint = pathfinder::math::reflectPointOverLine(vertices[14], vertices[12], vertices[0]);
    ASSERT_EQ(vertices[10], reflectedPoint);
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