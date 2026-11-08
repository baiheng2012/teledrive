#include <gtest/gtest.h>
#include <teledrive/common/vec2.hpp>

using teledrive::Vec2;

TEST(Vec2, AddAndSubtract) {
  Vec2 a{1, 2}, b{3, 5};
  EXPECT_EQ((a + b).x, 4);
  EXPECT_EQ((a + b).y, 7);
  EXPECT_EQ((b - a).x, 2);
  EXPECT_EQ((b - a).y, 3);
}

TEST(Vec2, DotProduct) {
  EXPECT_DOUBLE_EQ(Vec2(1, 0).dot(Vec2(0, 1)), 0.0);
  EXPECT_DOUBLE_EQ(Vec2(2, 3).dot(Vec2(4, 5)), 23.0);
}

TEST(Vec2, Norm) {
  EXPECT_DOUBLE_EQ(Vec2(3, 4).norm(), 5.0);
}
