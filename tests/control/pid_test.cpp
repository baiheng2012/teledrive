#include <gtest/gtest.h>
#include <teledrive/control/pid.hpp>

#include <cmath>
#include <limits>

using teledrive::control::Pid;
using teledrive::control::PidConfig;

namespace {
PidConfig cfg(double kp, double ki, double kd) {
  PidConfig c;
  c.kp = kp; c.ki = ki; c.kd = kd;
  c.out_min = -100.0; c.out_max = 100.0;
  return c;
}
}  // namespace

TEST(Pid, ProportionalOnly) {
  Pid pid(cfg(2.0, 0, 0));
  EXPECT_DOUBLE_EQ(pid.update(3.0, 0.0, 0.1), 6.0);
  EXPECT_DOUBLE_EQ(pid.update(-1.5, 0.0, 0.1), -3.0);
}

TEST(Pid, IntegralAccumulatesOverTime) {
  Pid pid(cfg(0, 1.0, 0));
  // 误差恒为 2，每步 0.5 秒：积分 = 1, 2, 3 ...
  EXPECT_NEAR(pid.update(2.0, 0.0, 0.5), 1.0, 1e-12);
  EXPECT_NEAR(pid.update(2.0, 0.0, 0.5), 2.0, 1e-12);
  EXPECT_NEAR(pid.update(2.0, 0.0, 0.5), 3.0, 1e-12);
}

TEST(Pid, OutputIsClamped) {
  PidConfig c = cfg(10.0, 0, 0);
  c.out_min = -1.0; c.out_max = 1.0;
  Pid pid(c);
  EXPECT_DOUBLE_EQ(pid.update(5.0, 0.0, 0.1), 1.0);
  EXPECT_DOUBLE_EQ(pid.update(-5.0, 0.0, 0.1), -1.0);
}

// 微分作用在测量值上：测量值上升，D 项给出负的修正（阻尼）
TEST(Pid, DerivativeActsOnMeasurementNotError) {
  Pid pid(cfg(0, 0, 1.0));
  pid.update(0.0, 0.0, 0.1);                    // 第一次：建立历史
  EXPECT_NEAR(pid.update(0.0, 1.0, 0.1), -10.0, 1e-9);  // 测量值 0→1，斜率 10，D = -10
}

// 目标突变（误差突变）而测量值不变：不应产生微分尖峰
TEST(Pid, NoDerivativeKickOnSetpointStep) {
  Pid pid(cfg(0, 0, 1.0));
  pid.update(0.0, 5.0, 0.1);
  EXPECT_NEAR(pid.update(100.0, 5.0, 0.1), 0.0, 1e-12);  // 误差跳到 100，输出仍为 0
}

TEST(Pid, FirstCallHasNoDerivativeSpike) {
  Pid pid(cfg(0, 0, 1.0));
  EXPECT_DOUBLE_EQ(pid.update(0.0, 1000.0, 0.1), 0.0);  // 没有"上一次"，不能算斜率
}

// 抗积分饱和：长时间饱和后误差反向，输出应该很快反向，而不是被巨大积分拖住
TEST(Pid, AntiWindupLetsOutputRecoverQuickly) {
  PidConfig c = cfg(1.0, 5.0, 0);
  c.out_min = -1.0; c.out_max = 1.0;
  Pid pid(c);

  for (int i = 0; i < 1000; ++i) pid.update(10.0, 0.0, 0.1);   // 长时间饱和在 +1
  EXPECT_LE(pid.integral(), 1.0 / 5.0 + 1e-9);                  // 积分没有无限增长

  // 误差突然变为 -1：P=-1，I 很小，输出应立即为负
  EXPECT_LT(pid.update(-1.0, 0.0, 0.1), 0.0);
}

TEST(Pid, ResetClearsState) {
  Pid pid(cfg(0, 1.0, 0));
  pid.update(5.0, 0.0, 1.0);
  ASSERT_GT(pid.integral(), 0.0);
  pid.reset();
  EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
}

TEST(Pid, InvalidDtIsIgnored) {
  Pid pid(cfg(1.0, 1.0, 0));
  const double first = pid.update(1.0, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0, 0.0), first);
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0, -1.0), first);
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0, std::numeric_limits<double>::quiet_NaN()), first);
  EXPECT_NEAR(pid.integral(), 0.1, 1e-12);  // 无效 dt 没有污染积分
}

TEST(Pid, NonFiniteErrorDoesNotPoisonState) {
  Pid pid(cfg(1.0, 1.0, 0));
  pid.update(1.0, 0.0, 0.1);
  const double before = pid.integral();
  pid.update(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.1);
  EXPECT_DOUBLE_EQ(pid.integral(), before);
  EXPECT_TRUE(std::isfinite(pid.update(1.0, 0.0, 0.1)));
}
