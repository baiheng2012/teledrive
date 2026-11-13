// 手写 Pid 与 Simulink 生成 Pid 的等价性：同一随机输入序列，逐点比较输出。
#include <gtest/gtest.h>
#include <teledrive/control/pid_core.hpp>
#include <teledrive/control/heading_hold.hpp>

#include <cmath>
#include <random>

using namespace teledrive::control;

namespace {
PidConfig cfg(double kp, double ki, double kd, double lo, double hi, double a) {
  PidConfig c;
  c.kp = kp; c.ki = ki; c.kd = kd; c.out_min = lo; c.out_max = hi; c.d_filter_alpha = a;
  return c;
}

// 随机序列上比较，返回最大绝对误差。seed 固定，保证可重复。
double max_diff(const PidConfig& c, unsigned seed, int n) {
  HandwrittenPid a(c);
  GeneratedPid b(c);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> d(-3.0, 3.0);
  double worst = 0.0;
  for (int i = 0; i < n; ++i) {
    const double e = d(rng), m = d(rng);
    worst = std::max(worst, std::abs(a.update(e, m, 0.02) - b.update(e, m, 0.02)));
  }
  return worst;
}
}  // namespace

TEST(GeneratedPid, MatchesHandwrittenOnRandomInputs) {
  EXPECT_LT(max_diff(cfg(2.0, 0.2, 0.1, -1.0, 1.0, 0.5), 1, 5000), 1e-9);
}

TEST(GeneratedPid, MatchesWhenHeavilySaturated) {      // 大增益：长期饱和，考验抗饱和
  EXPECT_LT(max_diff(cfg(10.0, 5.0, 0.5, -1.0, 1.0, 1.0), 2, 5000), 1e-9);
}

TEST(GeneratedPid, MatchesWithoutFilterAndWithPureP) {
  EXPECT_LT(max_diff(cfg(1.0, 0.0, 0.0, -100.0, 100.0, 1.0), 3, 2000), 1e-12);
}

TEST(GeneratedPid, ResetRestoresInitialBehaviour) {
  GeneratedPid g(cfg(0.0, 1.0, 0.0, -100.0, 100.0, 1.0));
  const double first = g.update(1.0, 0.0, 0.02);
  g.update(1.0, 0.0, 0.02);
  g.reset();
  EXPECT_DOUBLE_EQ(g.update(1.0, 0.0, 0.02), first);
}

TEST(GeneratedPid, InvalidInputsKeepLastOutput) {
  GeneratedPid g(cfg(1.0, 0.0, 0.0, -100.0, 100.0, 1.0));
  const double ok = g.update(1.0, 0.0, 0.02);
  EXPECT_DOUBLE_EQ(g.update(std::nan(""), 0.0, 0.02), ok);
  EXPECT_DOUBLE_EQ(g.update(1.0, 0.0, 0.0), ok);
}
