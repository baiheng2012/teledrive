#include <gtest/gtest.h>
#include <teledrive/sim/diff_drive.hpp>

#include <cmath>
#include <limits>

using teledrive::protocol::Command;
using teledrive::protocol::VehicleState;
using teledrive::sim::DiffDriveModel;
using teledrive::sim::kPi;
using teledrive::sim::VehicleParams;

namespace {

Command cmd(float linear, float angular) {
  Command c;
  c.linear = linear;
  c.angular = angular;
  return c;
}

// 重复执行 steps 步，每步 dt 秒
void run(DiffDriveModel& m, const Command& c, int steps, double dt) {
  for (int i = 0; i < steps; ++i) {
    m.step(c, dt);
  }
}

// "瞬时响应"的车：加速度极大，速度一步就到位。
// 用来把"加速度限制"这个因素排除掉，单独测几何运动和电量。
VehicleParams instant_params() {
  VehicleParams p;
  p.max_linear_speed = 10.0;
  p.max_angular_speed = 10.0;
  p.max_linear_accel = p.max_linear_decel = 1e9;
  p.max_angular_accel = p.max_angular_decel = 1e9;
  return p;
}

constexpr double kDt = 0.02;  // 50Hz 控制周期

}  // namespace

TEST(DiffDrive, StartsAtRestAtOriginWithFullBattery) {
  DiffDriveModel m;
  const auto& s = m.state();
  EXPECT_DOUBLE_EQ(s.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(s.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(s.pose.heading, 0.0);
  EXPECT_DOUBLE_EQ(s.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(s.battery_pct, 100.0);
}

// ---------- 加速度限制 ----------

TEST(DiffDrive, AccelerationIsRateLimited) {
  DiffDriveModel m;  // 默认加速度 1.0 m/s²
  m.step(cmd(1.0f, 0.0f), kDt);
  // 一步只能加速 a*dt = 0.02，而不是瞬间到 1.0
  EXPECT_NEAR(m.state().linear_velocity, 0.02, 1e-12);
}

TEST(DiffDrive, ReachesTargetSpeedAndCruisesAtConstantSpeed) {
  DiffDriveModel m;
  run(m, cmd(1.0f, 0.0f), 60, kDt);  // 1.2 秒，加速只需 1 秒
  EXPECT_NEAR(m.state().linear_velocity, 1.0, 1e-9);

  const double x0 = m.state().pose.position.x;
  run(m, cmd(1.0f, 0.0f), 100, kDt);  // 巡航 2 秒
  EXPECT_NEAR(m.state().pose.position.x - x0, 2.0, 1e-6);  // 1 m/s * 2 s = 2 m
  EXPECT_NEAR(m.state().pose.position.y, 0.0, 1e-12);       // 直行，不偏航
}

TEST(DiffDrive, BrakesHarderThanItAccelerates) {
  DiffDriveModel m;  // 加速 1.0，减速 2.0
  run(m, cmd(1.0f, 0.0f), 60, kDt);
  ASSERT_NEAR(m.state().linear_velocity, 1.0, 1e-9);

  run(m, cmd(0.0f, 0.0f), 10, kDt);  // 0.2 秒
  EXPECT_NEAR(m.state().linear_velocity, 1.0 - 2.0 * 0.2, 1e-9);  // 0.6

  run(m, cmd(0.0f, 0.0f), 15, kDt);  // 总共 0.5 秒，刚好停下
  EXPECT_NEAR(m.state().linear_velocity, 0.0, 1e-9);
}

TEST(DiffDrive, ReverseMovesBackward) {
  DiffDriveModel m;
  run(m, cmd(-0.5f, 0.0f), 100, kDt);
  EXPECT_LT(m.state().pose.position.x, 0.0);
  EXPECT_LT(m.state().linear_velocity, 0.0);
}

TEST(DiffDrive, PhysicalSpeedLimitClampsAbsurdTargets) {
  auto p = instant_params();
  p.max_linear_speed = 2.0;
  DiffDriveModel m(p);
  m.step(cmd(50.0f, 0.0f), kDt);  // 目标 50 m/s，物理上做不到
  EXPECT_DOUBLE_EQ(m.state().linear_velocity, 2.0);
}

// ---------- 几何：转弯与朝向 ----------

// 圆周运动：v = π m/s，ω = π rad/s，半径 R = v/ω = 1 m，周期 2 秒。
// 步长 0.01 s，正好 200 步走完一圈，应当回到起点、朝向回到 0。
// 这个测试能暴露积分方法的错误（用欧拉法会明显偏离）。
TEST(DiffDrive, FullCircleReturnsToStart) {
  DiffDriveModel m(instant_params());
  const float pi_f = static_cast<float>(kPi);  // Command 是 float，所以有微小的精度损失
  run(m, cmd(pi_f, pi_f), 200, 0.01);

  EXPECT_NEAR(m.state().pose.position.x, 0.0, 1e-4);
  EXPECT_NEAR(m.state().pose.position.y, 0.0, 1e-4);
  EXPECT_NEAR(m.state().pose.heading, 0.0, 1e-5);
}

TEST(DiffDrive, QuarterCircleLeftEndsUpAtExpectedPoint) {
  // 半径 1 m 的圆，向左转 90°：终点应在 (1, 1)，朝向 π/2
  DiffDriveModel m(instant_params());
  const float w = static_cast<float>(kPi / 2.0);  // 角速度 π/2 rad/s，用 1 秒转 90°
  run(m, cmd(w, w), 100, 0.01);                   // v = ω → R = 1 m

  EXPECT_NEAR(m.state().pose.position.x, 1.0, 1e-3);
  EXPECT_NEAR(m.state().pose.position.y, 1.0, 1e-3);
  EXPECT_NEAR(m.state().pose.heading, kPi / 2.0, 1e-5);
}

TEST(DiffDrive, HeadingStaysNormalized) {
  DiffDriveModel m(instant_params());
  // 以 3 rad/s 原地旋转 10 秒，累计转了 30 rad（约 5 圈），朝向必须始终在 (-π, π]
  for (int i = 0; i < 1000; ++i) {
    m.step(cmd(0.0f, 3.0f), 0.01);
    ASSERT_GT(m.state().pose.heading, -kPi);
    ASSERT_LE(m.state().pose.heading, kPi);
  }
}

// ---------- 防御：脏输入 ----------

TEST(DiffDrive, NonFiniteTargetsAreTreatedAsZero) {
  DiffDriveModel m(instant_params());
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  m.step(cmd(nan, inf), kDt);
  EXPECT_DOUBLE_EQ(m.state().linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(m.state().angular_velocity, 0.0);
  EXPECT_TRUE(std::isfinite(m.state().pose.position.x));
}

TEST(DiffDrive, InvalidTimeStepIsIgnored) {
  DiffDriveModel m(instant_params());
  m.step(cmd(1.0f, 0.0f), 0.0);
  m.step(cmd(1.0f, 0.0f), -0.02);
  m.step(cmd(1.0f, 0.0f), std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(m.state().linear_velocity, 0.0);  // 状态完全没变
  EXPECT_DOUBLE_EQ(m.state().battery_pct, 100.0);
}

// ---------- 电量 ----------

TEST(DiffDrive, BatteryDrainsWhileIdle) {
  auto p = instant_params();
  p.battery_idle_drain = 10.0;  // 夸张的值，让测试用很少步数就能看出变化
  DiffDriveModel m(p);
  run(m, cmd(0.0f, 0.0f), 10, 0.1);  // 共 1 秒
  EXPECT_NEAR(m.state().battery_pct, 90.0, 1e-9);
}

TEST(DiffDrive, MovingDrainsBatteryFasterThanIdle) {
  auto p = instant_params();
  p.battery_idle_drain = 0.0;
  p.battery_motion_drain_per_mps = 5.0;
  DiffDriveModel m(p);
  run(m, cmd(1.0f, 0.0f), 10, 0.1);  // 1 m/s 持续 1 秒，消耗 5%
  EXPECT_NEAR(m.state().battery_pct, 95.0, 1e-9);
}

TEST(DiffDrive, EmptyBatteryStopsTheVehicle) {
  auto p = instant_params();
  p.battery_idle_drain = 50.0;  // 2 秒耗尽
  DiffDriveModel m(p);
  run(m, cmd(1.0f, 0.0f), 30, 0.1);  // 3 秒，仍然一直下达前进指令

  EXPECT_DOUBLE_EQ(m.state().battery_pct, 0.0);       // 不会变成负数
  EXPECT_DOUBLE_EQ(m.state().linear_velocity, 0.0);   // 没电，不再响应指令
}

// ---------- 遥测 ----------

TEST(DiffDrive, TelemetryReflectsStateAndPassesThroughVehicleState) {
  DiffDriveModel m(instant_params());
  run(m, cmd(1.0f, 0.0f), 100, 0.01);  // 1 秒，前进 1 m

  const auto t = m.telemetry(VehicleState::Driving);
  EXPECT_NEAR(t.x, 1.0f, 1e-3f);
  EXPECT_NEAR(t.y, 0.0f, 1e-6f);
  EXPECT_NEAR(t.speed, 1.0f, 1e-6f);
  EXPECT_EQ(t.state, VehicleState::Driving);  // safety 层给什么，就原样填什么
  EXPECT_EQ(t.battery_pct, 100);              // 耗电很少，四舍五入仍是 100
}
