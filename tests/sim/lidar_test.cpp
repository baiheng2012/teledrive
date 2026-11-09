#include <gtest/gtest.h>
#include <teledrive/sim/lidar.hpp>

using teledrive::Vec2;
using teledrive::sim::Circle;
using teledrive::sim::Lidar;
using teledrive::sim::LidarConfig;
using teledrive::sim::kPi;
using teledrive::sim::Pose;
using teledrive::sim::World;

namespace {
// 默认 61 根射线、±90°：第 30 根正好指向正前方
constexpr std::size_t kCenter = 30;

World one_obstacle(double x, double y, double r = 0.5) {
  World w;
  w.obstacles.push_back(Circle{Vec2{x, y}, r});
  return w;
}
}  // namespace

TEST(Lidar, EmptyWorldReturnsMaxRangeEverywhere) {
  Lidar lidar;
  const auto s = lidar.scan(World{}, Pose{});
  ASSERT_EQ(s.count, 61u);
  for (std::size_t i = 0; i < s.count; ++i) {
    EXPECT_DOUBLE_EQ(s.ranges[i], s.max_range);
    EXPECT_FALSE(s.hit(i));
  }
}

TEST(Lidar, BeamAnglesSpanTheFieldOfView) {
  Lidar lidar;
  const auto s = lidar.scan(World{}, Pose{});
  EXPECT_NEAR(s.angle_at(0), -kPi / 2, 1e-12);
  EXPECT_NEAR(s.angle_at(kCenter), 0.0, 1e-12);
  EXPECT_NEAR(s.angle_at(s.count - 1), kPi / 2, 1e-12);
}

// 圆心 (5,0) 半径 0.5：正前方射线应在 x=4.5 处命中，距离 4.5
TEST(Lidar, CenterBeamHitsObstacleSurface) {
  LidarConfig c;
  c.max_range = 10.0;
  Lidar lidar(c);
  const auto s = lidar.scan(one_obstacle(5.0, 0.0), Pose{});
  EXPECT_NEAR(s.ranges[kCenter], 4.5, 1e-9);
  EXPECT_TRUE(s.hit(kCenter));
}

TEST(Lidar, ObstacleBeyondMaxRangeIsNotSeen) {
  Lidar lidar;  // max_range = 5，障碍物表面在 7.5
  const auto s = lidar.scan(one_obstacle(8.0, 0.0), Pose{});
  EXPECT_FALSE(s.hit(kCenter));
}

TEST(Lidar, ObstacleBehindIsNotSeen) {
  LidarConfig c;
  c.max_range = 10.0;
  Lidar lidar(c);
  const auto s = lidar.scan(one_obstacle(-5.0, 0.0), Pose{});
  for (std::size_t i = 0; i < s.count; ++i) EXPECT_FALSE(s.hit(i));
}

// 射线随车头转动：车头朝 +y（heading=π/2），障碍物在 (0,5)，正前方射线应命中
TEST(Lidar, RaysRotateWithVehicleHeading) {
  LidarConfig c;
  c.max_range = 10.0;
  Lidar lidar(c);
  Pose p;
  p.heading = kPi / 2;
  const auto s = lidar.scan(one_obstacle(0.0, 5.0), p);
  EXPECT_NEAR(s.ranges[kCenter], 4.5, 1e-9);
}

TEST(Lidar, RaysStartFromVehiclePosition) {
  LidarConfig c;
  c.max_range = 10.0;
  Lidar lidar(c);
  Pose p;
  p.position = Vec2{2.0, 0.0};
  const auto s = lidar.scan(one_obstacle(5.0, 0.0), p);
  EXPECT_NEAR(s.ranges[kCenter], 2.5, 1e-9);
}

// 射线起点在障碍物内部：距离 0（已经撞上了）
TEST(Lidar, OriginInsideObstacleReturnsZero) {
  Lidar lidar;
  const auto s = lidar.scan(one_obstacle(0.1, 0.0), Pose{});
  EXPECT_DOUBLE_EQ(s.ranges[kCenter], 0.0);
}

// 擦边而过的射线不应命中：圆心 (5, 2)，半径 0.5，正前方射线（y=0）离圆心 2m，肯定不中
TEST(Lidar, MissesObstacleOffToTheSide) {
  LidarConfig c;
  c.max_range = 10.0;
  Lidar lidar(c);
  const auto s = lidar.scan(one_obstacle(5.0, 2.0), Pose{});
  EXPECT_FALSE(s.hit(kCenter));
}

TEST(Lidar, NearestOfSeveralObstaclesWins) {
  LidarConfig c;
  c.max_range = 10.0;
  Lidar lidar(c);
  World w;
  w.obstacles.push_back(Circle{Vec2{8.0, 0.0}, 0.5});
  w.obstacles.push_back(Circle{Vec2{4.0, 0.0}, 0.5});
  const auto s = lidar.scan(w, Pose{});
  EXPECT_NEAR(s.ranges[kCenter], 3.5, 1e-9);
}

TEST(Lidar, BeamCountIsClamped) {
  LidarConfig c;
  c.beams = 100000;
  EXPECT_EQ(Lidar(c).scan(World{}, Pose{}).count, teledrive::sim::kMaxBeams);
  c.beams = 0;
  EXPECT_EQ(Lidar(c).scan(World{}, Pose{}).count, 1u);
}
