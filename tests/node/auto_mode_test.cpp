// 集成：操作端只发"arm + 自动"标志（速度为 0），车辆端自己沿路径行驶。
#include <gtest/gtest.h>
#include <teledrive/node/operator_session.hpp>
#include <teledrive/node/vehicle_node.hpp>
#include <teledrive/transport/mock_transport.hpp>

using namespace std::chrono_literals;
using teledrive::FakeClock;
using teledrive::Vec2;
using teledrive::node::OperatorSession;
using teledrive::node::VehicleNode;
using teledrive::node::VehicleNodeConfig;
using teledrive::transport::Endpoint;
using teledrive::transport::MockTransport;
namespace proto = teledrive::protocol;

namespace {
const Endpoint kOp{0x7F000001u, 9000};
const Endpoint kVeh{0x7F000001u, 9001};

struct Rig {
  FakeClock clock;
  MockTransport op_t{kOp};
  MockTransport veh_t{kVeh};
  VehicleNode vehicle;
  OperatorSession op{op_t, clock, kVeh};

  explicit Rig(VehicleNodeConfig cfg) : vehicle(veh_t, clock, cfg) {
    op_t.connect(&veh_t);
    veh_t.connect(&op_t);
  }

  void run(int n, std::uint8_t flags) {
    proto::Command c;       // 速度全为 0：自动模式下速度不来自操作员
    c.flags = flags;
    for (int i = 0; i < n; ++i) {
      op.send_command(c);
      vehicle.tick();
      op.poll();
      clock.advance(20ms);
    }
  }
};

constexpr std::uint8_t kArmAuto = proto::kFlagArm | proto::kFlagAuto;
}  // namespace

TEST(AutoModeNode, VehicleFollowsPathWithoutOperatorVelocity) {
  VehicleNodeConfig cfg;
  cfg.path = {Vec2{0, 0}, Vec2{3, 0}};
  Rig rig(cfg);

  rig.run(1, proto::kFlagArm);   // 先 arm（Idle -> Armed）
  rig.run(700, kArmAuto);        // 14 秒，足够走 3 米

  EXPECT_EQ(rig.vehicle.safety().state(), proto::VehicleState::Driving);
  EXPECT_TRUE(rig.vehicle.auto_active());
  EXPECT_TRUE(rig.vehicle.path_finished());
  EXPECT_NEAR(rig.vehicle.model().state().pose.position.x, 3.0, 0.3);
}

// 自动模式下前方有障碍物：限速器照样刹停，自动驾驶不能撞上去
TEST(AutoModeNode, StopsForObstacleOnThePath) {
  VehicleNodeConfig cfg;
  cfg.path = {Vec2{0, 0}, Vec2{8, 0}};
  cfg.world.obstacles.push_back(teledrive::sim::Circle{Vec2{5.0, 0.0}, 0.5});
  Rig rig(cfg);

  rig.run(1, proto::kFlagArm);
  rig.run(1500, kArmAuto);       // 30 秒

  const double x = rig.vehicle.model().state().pose.position.x;
  EXPECT_GT(x, 3.0);
  EXPECT_LT(x, 4.2);             // 停在障碍物前，没有撞上
  EXPECT_TRUE(rig.vehicle.obstacle_limited());
  EXPECT_FALSE(rig.vehicle.path_finished());
}

// 操作员松开自动标志：自动驾驶立刻停止，车刹停
TEST(AutoModeNode, DroppingAutoFlagStopsTheVehicle) {
  VehicleNodeConfig cfg;
  cfg.path = {Vec2{0, 0}, Vec2{20, 0}};
  Rig rig(cfg);

  rig.run(1, proto::kFlagArm);
  rig.run(100, kArmAuto);        // 开了 2 秒
  ASSERT_GT(rig.vehicle.model().state().linear_velocity, 0.3);

  rig.run(100, proto::kFlagArm); // 退出自动
  EXPECT_EQ(rig.vehicle.safety().state(), proto::VehicleState::Armed);
  EXPECT_FALSE(rig.vehicle.auto_active());
  EXPECT_NEAR(rig.vehicle.model().state().linear_velocity, 0.0, 1e-6);
}

// 自动模式下操作端失联：看门狗照常生效
TEST(AutoModeNode, LinkLossFaultsEvenInAutoMode) {
  VehicleNodeConfig cfg;
  cfg.path = {Vec2{0, 0}, Vec2{20, 0}};
  Rig rig(cfg);

  rig.run(1, proto::kFlagArm);
  rig.run(100, kArmAuto);
  for (int i = 0; i < 25; ++i) {   // 操作端沉默 0.5 秒
    rig.vehicle.tick();
    rig.clock.advance(20ms);
  }
  EXPECT_EQ(rig.vehicle.safety().state(), proto::VehicleState::Fault);
}
