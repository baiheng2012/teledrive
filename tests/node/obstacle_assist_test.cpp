// 集成：整条链路（操作端 → UDP 模拟 → 安全层 → 避障限速 → 车辆）一起验证。
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

VehicleNodeConfig config_with_obstacle(bool assist) {
  VehicleNodeConfig c;
  c.world.obstacles.push_back(teledrive::sim::Circle{Vec2{5.0, 0.0}, 0.5});
  c.obstacle_assist_enabled = assist;
  return c;
}

// 每个测试一份独立环境，通过构造参数决定辅助是否启用
struct Rig {
  FakeClock clock;
  MockTransport op_t{kOp};
  MockTransport veh_t{kVeh};
  VehicleNode vehicle;
  OperatorSession op{op_t, clock, kVeh};

  explicit Rig(bool assist) : vehicle(veh_t, clock, config_with_obstacle(assist)) {
    op_t.connect(&veh_t);
    veh_t.connect(&op_t);
  }

  void run(int n, float linear) {
    proto::Command c;
    c.linear = linear;
    c.flags = proto::kFlagArm;
    for (int i = 0; i < n; ++i) {
      op.send_command(c);
      vehicle.tick();
      op.poll();
      clock.advance(20ms);
    }
  }
};
}  // namespace

TEST(ObstacleAssistNode, VehicleStopsBeforeObstacleWhileOperatorHoldsThrottle) {
  Rig rig(true);
  rig.run(1500, 1.0f);   // 30 秒满油门

  EXPECT_EQ(rig.vehicle.safety().state(), proto::VehicleState::Driving);  // 安全层没觉得有问题
  EXPECT_TRUE(rig.vehicle.obstacle_limited());                              // 但辅助在介入
  const double x = rig.vehicle.model().state().pose.position.x;
  EXPECT_GT(x, 3.0);
  EXPECT_LT(x, 4.2);
}

// 对照组：关掉辅助，同样的操作会撞上（x 超过障碍物表面 4.5），证明上面的测试确实是辅助的功劳
TEST(ObstacleAssistNode, WithoutAssistVehicleDrivesIntoObstacle) {
  Rig rig(false);
  rig.run(1000, 1.0f);
  EXPECT_GT(rig.vehicle.model().state().pose.position.x, 4.5);
  EXPECT_FALSE(rig.vehicle.obstacle_limited());
}
