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