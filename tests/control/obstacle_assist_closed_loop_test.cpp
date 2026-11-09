// 闭环：车辆模型 + 雷达 + 限速器。操作员"一直推满油门"，验证车会自己停在障碍物前。
#include <gtest/gtest.h>
#include <teledrive/control/speed_limiter.hpp>
#include <teledrive/sim/diff_drive.hpp>

#include <algorithm>

using teledrive::Vec2;
using teledrive::control::SpeedLimiterConfig;
using teledrive::control::limit_for_obstacles;
using teledrive::sim::Circle;
using teledrive::sim::DiffDriveModel;
using teledrive::sim::Lidar;
using teledrive::sim::World;
namespace proto = teledrive::protocol;

namespace {

constexpr double kDt = 0.02;

struct Rig {
  World world;
  Lidar lidar;
  DiffDriveModel car;
  SpeedLimiterConfig cfg;
  double min_clearance{1e9};   // 全程车心到障碍物表面的最小距离

  Rig() { world.obstacles.push_back(Circle{Vec2{5.0, 0.0}, 0.5}); }

  // 一个控制周期：操作员给出 (linear, angular)，辅助处理后交给车
  void tick(float linear, float angular) {
    proto::Command op;
    op.linear = linear;
    op.angular = angular;
    const auto scan = lidar.scan(world, car.state().pose);
    car.step(limit_for_obstacles(op, scan, cfg).command, kDt);
    for (const auto& o : world.obstacles) {
      min_clearance =
          std::min(min_clearance, (car.state().pose.position - o.center).norm() - o.radius);
    }
  }
  void run(int n, float linear, float angular) {
    for (int i = 0; i < n; ++i) tick(linear, angular);
  }
};

}  // namespace

TEST(ObstacleAssist, StopsBeforeObstacleUnderFullThrottle) {
  Rig rig;
  rig.run(1500, 1.0f, 0.0f);   // 30 秒一直推满油门

  const double x = rig.car.state().pose.position.x;
  // 理论停车位置：障碍物表面 x=4.5，减去 stop_distance 0.6 → 3.9
  EXPECT_GT(x, 3.0);                                 // 没有过早停住
  EXPECT_LT(x, 4.2);                                 // 没有冲过头
  EXPECT_LT(std::abs(rig.car.state().linear_velocity), 0.05);  // 确实停下了
  EXPECT_GT(rig.min_clearance, 0.4);                 // 全程没有逼近到危险距离
}

// 操作员在障碍物前停住后，应该能转头离开，再继续前进（辅助不能把车"锁死"）
TEST(ObstacleAssist, OperatorCanTurnAwayAndContinue) {
  Rig rig;
  rig.run(1000, 1.0f, 0.0f);                          // 先开到障碍物前停住
  const Vec2 stopped = rig.car.state().pose.position;
  ASSERT_LT(std::abs(rig.car.state().linear_velocity), 0.05);

  rig.run(100, 0.0f, 1.0f);                           // 原地左转约 2 秒
  rig.run(100, 1.0f, 0.0f);                           // 朝新方向前进 2 秒

  const double moved = (rig.car.state().pose.position - stopped).norm();
  EXPECT_GT(moved, 1.0);                              // 确实开走了
  EXPECT_GT(rig.min_clearance, 0.4);                  // 全程仍然没有靠近障碍物
}

// 斜着靠近、擦着走廊边缘的情形：车不应撞上障碍物
TEST(ObstacleAssist, NeverCollidesWhileSteeringTowardObstacle) {
  Rig rig;
  for (int i = 0; i < 1500; ++i) {
    // 操作员一边前进，一边持续轻微右转，轨迹会弯向障碍物
    rig.tick(1.0f, i < 200 ? 0.0f : -0.15f);
  }
  EXPECT_GT(rig.min_clearance, 0.1);   // 无论怎么走，都没有碰上（clearance > 0 即未接触）
}
