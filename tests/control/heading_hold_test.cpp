#include <gtest/gtest.h>
#include <teledrive/control/heading_hold.hpp>
#include <teledrive/sim/diff_drive.hpp>

#include <cmath>

using teledrive::control::HeadingHold;
using teledrive::control::angle_diff;
using teledrive::sim::DiffDriveModel;
using teledrive::sim::kPi;
using teledrive::sim::VehicleParams;
namespace proto = teledrive::protocol;

TEST(AngleDiff, SimpleCases) {
  EXPECT_NEAR(angle_diff(0.5, 0.2), 0.3, 1e-12);
  EXPECT_NEAR(angle_diff(0.2, 0.5), -0.3, 1e-12);
}

// 回绕：从 179° 到 -179° 应该是 +2°，不是 -358°
TEST(AngleDiff, TakesShortestPathAcrossPi) {
  const double d2r = kPi / 180.0;
  EXPECT_NEAR(angle_diff(-179 * d2r, 179 * d2r), 2 * d2r, 1e-12);
  EXPECT_NEAR(angle_diff(179 * d2r, -179 * d2r), -2 * d2r, 1e-12);
}

TEST(HeadingHold, PassesThroughOperatorSteering) {
  HeadingHold hh;
  EXPECT_DOUBLE_EQ(hh.update(0.8, 0.0, 1.0, 0.02), 0.8);
  EXPECT_FALSE(hh.holding());
}

TEST(HeadingHold, DoesNothingWhenStationary) {
  HeadingHold hh;
  EXPECT_DOUBLE_EQ(hh.update(0.0, 1.0, 0.0, 0.02), 0.0);
  EXPECT_FALSE(hh.holding());
}

TEST(HeadingHold, CorrectsDriftBackToTarget) {
  HeadingHold hh;
  hh.update(0.0, 0.0, 1.0, 0.02);                  // 开始直行，锁定朝向 0
  ASSERT_TRUE(hh.holding());
  EXPECT_LT(hh.update(0.0, 0.1, 1.0, 0.02), 0.0);  // 偏左了 → 向右修正（负）
  EXPECT_GT(hh.update(0.0, -0.1, 1.0, 0.02), 0.0); // 偏右了 → 向左修正（正）
}

// 闭环：给车施加持续的"侧向扰动"（模拟单侧轮子打滑），
// 有航向保持时朝向误差应保持很小；没有时会一直偏下去。
TEST(HeadingHold, RejectsConstantDisturbanceInClosedLoop) {
  constexpr double kDt = 0.02;
  constexpr double kDisturbance = 0.05;  // rad/s 的恒定偏航扰动

  auto simulate = [&](bool with_hold) {
    VehicleParams p;
    DiffDriveModel car(p);
    HeadingHold hh;
    double max_err = 0.0;
    for (int i = 0; i < 1500; ++i) {  // 30 秒
      const auto& s = car.state();
      double w_cmd = 0.0;
      if (with_hold) w_cmd = hh.update(0.0, s.pose.heading, s.linear_velocity, kDt);
      proto::Command c;
      c.linear = 1.0f;
      c.angular = static_cast<float>(w_cmd + kDisturbance);  // 扰动叠加在指令上
      car.step(c, kDt);
      if (i > 250) max_err = std::max(max_err, std::abs(car.state().pose.heading));
    }
    return max_err;
  };

  const double without = simulate(false);
  const double with = simulate(true);
  EXPECT_GT(without, 1.0);   // 没有辅助：朝向严重偏离
  EXPECT_LT(with, 0.15);     // 有辅助：偏差很小
}
