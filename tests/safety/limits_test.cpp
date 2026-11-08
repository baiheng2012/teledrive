#include <gtest/gtest.h>
#include <teledrive/safety/limits.hpp>

#include <limits>

using teledrive::protocol::Command;
using teledrive::safety::SafetyLimits;
using teledrive::safety::sanitize;

namespace {
Command make(float lin, float ang, std::uint8_t flags = 0) {
  Command c;
  c.linear = lin;
  c.angular = ang;
  c.flags = flags;
  return c;
}
const SafetyLimits kLimits{1.0f, 2.0f};  // 线速度 ±1.0，角速度 ±2.0
}  // namespace

TEST(Limits, PassesValuesWithinRangeUnchanged) {
  auto r = sanitize(make(0.5f, -1.0f, 0x02), kLimits);
  ASSERT_TRUE(r.has_value());
  EXPECT_FLOAT_EQ(r->linear, 0.5f);
  EXPECT_FLOAT_EQ(r->angular, -1.0f);
  EXPECT_EQ(r->flags, 0x02);  // flags 原样保留
}

TEST(Limits, ClampsPositiveOverrange) {
  auto r = sanitize(make(5.0f, 9.0f), kLimits);
  ASSERT_TRUE(r.has_value());
  EXPECT_FLOAT_EQ(r->linear, 1.0f);
  EXPECT_FLOAT_EQ(r->angular, 2.0f);
}

TEST(Limits, ClampsNegativeOverrange) {
  auto r = sanitize(make(-5.0f, -9.0f), kLimits);
  ASSERT_TRUE(r.has_value());
  EXPECT_FLOAT_EQ(r->linear, -1.0f);
  EXPECT_FLOAT_EQ(r->angular, -2.0f);
}

TEST(Limits, RejectsNaN) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(sanitize(make(nan, 0.0f), kLimits).has_value());
  EXPECT_FALSE(sanitize(make(0.0f, nan), kLimits).has_value());
}

TEST(Limits, RejectsInfinity) {
  const float inf = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(sanitize(make(inf, 0.0f), kLimits).has_value());
  EXPECT_FALSE(sanitize(make(0.0f, -inf), kLimits).has_value());
}
