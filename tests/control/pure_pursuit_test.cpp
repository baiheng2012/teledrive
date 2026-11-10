#include <gtest/gtest.h>
#include <teledrive/control/pure_pursuit.hpp>
#include <teledrive/sim/diff_drive.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using teledrive::Vec2;
using teledrive::control::PurePursuit;
using teledrive::control::PurePursuitConfig;
using teledrive::sim::DiffDriveModel;
using teledrive::sim::Pose;
namespace proto = teledrive::protocol;

namespace {

Pose at(double x, double y, double heading) {
  Pose p;
  p.position = Vec2{x, y};
  p.heading = heading;
  return p;
}

PurePursuit make(std::vector<Vec2> path) {
  PurePursuit pp;
  pp.set_path(std::move(path));
  return pp;
}

}  // namespace

// ---------- 单元测试：每次只看一个决策 ----------

TEST(PurePursuit, NoPathMeansFinishedWithZeroOutput) {
  PurePursuit pp;
  const auto o = pp.update(at(0, 0, 0));
  EXPECT_TRUE(o.finished);
  EXPECT_DOUBLE_EQ(o.linear, 0.0);
  EXPECT_DOUBLE_EQ(o.angular, 0.0);

  pp.set_path({Vec2{1, 1}});   // 只有一个点，不构成路径
  EXPECT_TRUE(pp.update(at(0, 0, 0)).finished);
}

TEST(PurePursuit, DrivesStraightWhenAlignedWithPath) {
  auto pp = make({Vec2{0, 0}, Vec2{10, 0}});
  const auto o = pp.update(at(0, 0, 0));
  EXPECT_FALSE(o.finished);
  EXPECT_NEAR(o.angular, 0.0, 1e-9);
  EXPECT_NEAR(o.linear, 0.6, 1e-9);        // 离终点还远，按巡航速度
  EXPECT_NEAR(o.lookahead.x, 0.8, 1e-9);   // 追逐点在路径上前方 Ld 处
}

// 路径在车的左边 -> 应当左转（正角速度）；在右边 -> 右转
TEST(PurePursuit, SteersTowardPathOnTheLeft) {
  auto pp = make({Vec2{0, 0}, Vec2{10, 0}});
  EXPECT_GT(pp.update(at(0, -1, 0)).angular, 0.0);
}

TEST(PurePursuit, SteersTowardPathOnTheRight) {
  auto pp = make({Vec2{0, 0}, Vec2{10, 0}});
  EXPECT_LT(pp.update(at(0, 1, 0)).angular, 0.0);
}

// 追逐点在车后面：不能边前进边画大圈，应当原地转向。
// 车头朝 3.0 rad（约 172°），路径朝 +x：最短转向方向是顺时针（负）。
TEST(PurePursuit, RotatesInPlaceWhenTargetIsBehind) {
  auto pp = make({Vec2{0, 0}, Vec2{5, 0}});
  const auto o = pp.update(at(0, 0, 3.0));
  EXPECT_DOUBLE_EQ(o.linear, 0.0);
  EXPECT_LT(o.angular, 0.0);
}

TEST(PurePursuit, SlowsDownNearTheGoal) {
  auto pp = make({Vec2{0, 0}, Vec2{10, 0}});
  const auto o = pp.update(at(9.7, 0, 0));   // 离终点 0.3m
  EXPECT_NEAR(o.linear, 0.3, 1e-9);          // v = 1.0 * 0.3
}

TEST(PurePursuit, FinishesWithinGoalToleranceAndStaysFinished) {
  auto pp = make({Vec2{0, 0}, Vec2{10, 0}});
  const auto o = pp.update(at(9.95, 0, 0));
  EXPECT_TRUE(o.finished);
  EXPECT_DOUBLE_EQ(o.linear, 0.0);

  // 完成状态是"锁存"的：车被推开以后也不会自己重新出发，除非 restart
  EXPECT_TRUE(pp.update(at(2.0, 0, 0)).finished);
  EXPECT_TRUE(pp.finished());
  pp.restart(at(2.0, 0, 0));
  EXPECT_FALSE(pp.update(at(2.0, 0, 0)).finished);
}

// 起点和终点几乎重合的路径：一开始不能被判为"到达终点"，
// 因为车还在第一段，并没有真的走完。
TEST(PurePursuit, DoesNotFinishImmediatelyOnLoopLikePath) {
  auto pp = make({Vec2{0, 0}, Vec2{2, 0}, Vec2{2, 2}, Vec2{0, 2}, Vec2{0, 0.05}});
  const auto o = pp.update(at(0, 0, 0));
  EXPECT_FALSE(o.finished);
  EXPECT_GT(o.linear, 0.0);
}

TEST(PurePursuit, RestartChoosesNearestSegment) {
  auto pp = make({Vec2{0, 0}, Vec2{5, 0}, Vec2{5, 5}});
  pp.restart(at(5, 3, 1.57));      // 车在第二段上
  EXPECT_EQ(pp.segment(), 1u);
  pp.restart(at(1, 0.1, 0));       // 车在第一段上
  EXPECT_EQ(pp.segment(), 0u);
}

// 路径进度只增不减：车靠近第二段后，即使又靠近第一段，也不会回到第一段
TEST(PurePursuit, ProgressNeverGoesBackward) {
  auto pp = make({Vec2{0, 0}, Vec2{4, 0}, Vec2{4, 4}});
  pp.update(at(3.6, 0.5, 1.5));    // 离第二段更近 -> 前进到第 1 段
  ASSERT_EQ(pp.segment(), 1u);
  pp.update(at(2.0, 0.1, 0.0));    // 位置离第一段更近
  EXPECT_EQ(pp.segment(), 1u);     // 仍然停在第 1 段
}

// ---------- 闭环：和真实车辆模型（有加速度限制）一起跑 ----------

namespace {

struct RunResult {
  bool finished{false};
  double max_cross_track{0.0};
  Vec2 end{};
};

RunResult simulate(const std::vector<Vec2>& path, double max_seconds) {
  constexpr double kDt = 0.02;
  PurePursuit pp;
  pp.set_path(path);
  DiffDriveModel car;               // 默认：位于原点，朝向 0
  pp.restart(car.state().pose);

  RunResult r;
  for (int i = 0; i < static_cast<int>(max_seconds / kDt); ++i) {
    const auto o = pp.update(car.state().pose);
    r.max_cross_track = std::max(r.max_cross_track, o.cross_track);
    if (o.finished) {
      r.finished = true;
      break;
    }
    proto::Command c;
    c.linear = static_cast<float>(o.linear);
    c.angular = static_cast<float>(o.angular);
    car.step(c, kDt);
  }
  r.end = car.state().pose.position;
  return r;
}

}  // namespace

// 车在原点，路径是 y=-1 的直线：车要先"并线"，再沿线行驶到终点
TEST(PurePursuitClosedLoop, ConvergesOntoOffsetStraightPath) {
  const auto r = simulate({Vec2{0, -1}, Vec2{10, -1}}, 60.0);
  EXPECT_TRUE(r.finished);
  EXPECT_LT((r.end - Vec2{10, -1}).norm(), 0.25);
}

// 方形路径：要经过三个 90° 转角。阈值放宽（1.0 m），主要验证"能走完，不迷路"。
TEST(PurePursuitClosedLoop, FollowsSquarePathToTheEnd) {
  const auto r = simulate({Vec2{0, 0}, Vec2{4, 0}, Vec2{4, 4}, Vec2{0, 4}}, 120.0);
  EXPECT_TRUE(r.finished);
  EXPECT_LT((r.end - Vec2{0, 4}).norm(), 0.3);
  EXPECT_LT(r.max_cross_track, 1.0);
}
