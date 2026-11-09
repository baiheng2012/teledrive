#include <gtest/gtest.h>
#include <teledrive/control/speed_limiter.hpp>

#include <cmath>

using teledrive::Vec2;
using teledrive::control::AssistResult;
using teledrive::control::SpeedLimiterConfig;
using teledrive::control::allowed_forward_speed;
using teledrive::control::kInf;
using teledrive::control::limit_for_obstacles;
using teledrive::control::nearest_obstacle_ahead;
using teledrive::sim::Circle;
using teledrive::sim::Lidar;
using teledrive::sim::LidarScan;
using teledrive::sim::Pose;
using teledrive::sim::World;
namespace proto = teledrive::protocol;

namespace {

LidarScan scan_of(double x, double y, double r = 0.5) {
  World w;
  w.obstacles.push_back(Circle{Vec2{x, y}, r});
  return Lidar{}.scan(w, Pose{});
}

proto::Command cmd(float lin, float ang = 0.0f, std::uint8_t flags = 0) {
  proto::Command c;
  c.linear = lin; c.angular = ang; c.flags = flags;
  return c;
}

const SpeedLimiterConfig kCfg{};  // 走廊半宽 0.45，stop 0.6，decel 1.0

}  // namespace

// ---------- 允许速度公式 ----------

TEST(AllowedSpeed, NoObstacleMeansNoLimit) {
  EXPECT_TRUE(std::isinf(allowed_forward_speed(kInf, kCfg)));
}

TEST(AllowedSpeed, ZeroWhenWithinStopDistance) {
  EXPECT_DOUBLE_EQ(allowed_forward_speed(0.6, kCfg), 0.0);
  EXPECT_DOUBLE_EQ(allowed_forward_speed(0.3, kCfg), 0.0);
}

TEST(AllowedSpeed, FollowsBrakingDistanceFormula) {
  // d=1.6 → 可用刹车距离 1.0 → v = sqrt(2*1.0*1.0) = √2
  EXPECT_NEAR(allowed_forward_speed(1.6, kCfg), std::sqrt(2.0), 1e-12);
}

TEST(AllowedSpeed, IncreasesWithDistance) {
  EXPECT_LT(allowed_forward_speed(1.0, kCfg), allowed_forward_speed(2.0, kCfg));
}

// ---------- 走廊判定 ----------

TEST(Corridor, FindsObstacleStraightAhead) {
  // 圆心 (2.1,0) 半径 0.5：最近点 x=1.6
  EXPECT_NEAR(nearest_obstacle_ahead(scan_of(2.1, 0.0), kCfg), 1.6, 1e-9);
}

TEST(Corridor, IgnoresObstacleBesideTheLane) {
  // 圆心 (2,2)，最近点离中线 1.5m，远大于走廊半宽 0.45
  EXPECT_TRUE(std::isinf(nearest_obstacle_ahead(scan_of(2.0, 2.0), kCfg)));
}

TEST(Corridor, IgnoresEmptyWorld) {
  EXPECT_TRUE(std::isinf(nearest_obstacle_ahead(Lidar{}.scan(World{}, Pose{}), kCfg)));
}

// ---------- 对指令的处理 ----------

TEST(LimitForObstacles, PassesThroughWhenPathIsClear) {
  const auto r = limit_for_obstacles(cmd(1.0f), Lidar{}.scan(World{}, Pose{}), kCfg);
  EXPECT_FLOAT_EQ(r.command.linear, 1.0f);
  EXPECT_FALSE(r.limited);
}

TEST(LimitForObstacles, ClampsSpeedWhenObstacleAhead) {
  const auto r = limit_for_obstacles(cmd(2.0f), scan_of(2.1, 0.0), kCfg);  // 允许 √2
  EXPECT_NEAR(r.command.linear, std::sqrt(2.0), 1e-3);
  EXPECT_TRUE(r.limited);
  EXPECT_NEAR(r.nearest, 1.6, 1e-9);
}

// 操作员本来就开得慢：辅助不介入，更不会把速度"加"上去
TEST(LimitForObstacles, NeverIncreasesOperatorSpeed) {
  const auto r = limit_for_obstacles(cmd(0.3f), scan_of(2.1, 0.0), kCfg);
  EXPECT_FLOAT_EQ(r.command.linear, 0.3f);
  EXPECT_FALSE(r.limited);
}

TEST(LimitForObstacles, StopsCompletelyWhenTooClose) {
  const auto r = limit_for_obstacles(cmd(1.0f), scan_of(1.0, 0.0), kCfg);  // 最近点 0.5 < 0.6
  EXPECT_FLOAT_EQ(r.command.linear, 0.0f);
  EXPECT_TRUE(r.limited);
}

// 后退不受限制（雷达在前面，对后方一无所知）
TEST(LimitForObstacles, ReverseIsNotLimited) {
  const auto r = limit_for_obstacles(cmd(-0.8f), scan_of(1.0, 0.0), kCfg);
  EXPECT_FLOAT_EQ(r.command.linear, -0.8f);
  EXPECT_FALSE(r.limited);
}

// 转向和标志位不动：操作员必须能原地转走脱困，急停/arm 标志更不能被改
TEST(LimitForObstacles, DoesNotTouchAngularOrFlags) {
  const auto r = limit_for_obstacles(cmd(1.0f, 0.9f, proto::kFlagArm), scan_of(1.0, 0.0), kCfg);
  EXPECT_FLOAT_EQ(r.command.angular, 0.9f);
  EXPECT_EQ(r.command.flags, proto::kFlagArm);
}
