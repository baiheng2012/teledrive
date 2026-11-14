// VehicleNode 与 OperatorSession 一行不改，换成 CanTransport 就能跑在 CAN 总线上。
#include <gtest/gtest.h>
#include <teledrive/can/can_transport.hpp>
#include <teledrive/can/mock_can_bus.hpp>
#include <teledrive/node/operator_session.hpp>
#include <teledrive/node/vehicle_node.hpp>

using namespace std::chrono_literals;
using namespace teledrive;
using teledrive::can::can_endpoint;
namespace proto = teledrive::protocol;

namespace {

struct CanRig {
  FakeClock clock;
  can::MockCanBus veh_bus, op_bus;
  can::CanTransport veh_t{veh_bus, 1}, op_t{op_bus, 2};
  node::VehicleNode vehicle{veh_t, clock};
  node::OperatorSession op{op_t, clock, can_endpoint(1)};

  CanRig() { can::MockCanBus::join(veh_bus, op_bus); }

  void run(int n, float linear, std::uint8_t flags) {
    proto::Command c;
    c.linear = linear;
    c.flags = flags;
    for (int i = 0; i < n; ++i) {
      op.send_command(c);
      vehicle.tick();
      op.poll();
      clock.advance(20ms);
    }
  }
};

}  // namespace

TEST(CanNode, OperatorDrivesVehicleOverCan) {
  CanRig r;
  r.run(2, 0.0f, proto::kFlagArm);
  r.run(60, 0.5f, proto::kFlagArm);
  EXPECT_EQ(r.vehicle.safety().state(), proto::VehicleState::Driving);
  EXPECT_GT(r.vehicle.model().state().pose.position.x, 0.2);
  ASSERT_TRUE(r.op.latest_telemetry().has_value());   // 遥测 5 个 CAN 帧一条，也到了
  EXPECT_EQ(r.op.latest_telemetry()->state, proto::VehicleState::Driving);
}

TEST(CanNode, EStopOverCanFaultsVehicle) {
  CanRig r;
  r.run(2, 0.0f, proto::kFlagArm);
  r.run(40, 0.5f, proto::kFlagArm);
  r.run(1, 0.5f, proto::kFlagArm | proto::kFlagEStop);
  EXPECT_EQ(r.vehicle.safety().state(), proto::VehicleState::Fault);
}

// 偶尔丢两个 CAN 帧（约丢掉一条指令），远小于 200ms 看门狗，车不受影响
TEST(CanNode, ShortFrameLossIsTolerated) {
  CanRig r;
  r.run(2, 0.0f, proto::kFlagArm);
  r.run(40, 0.5f, proto::kFlagArm);
  r.op_bus.drop_next_frames(2);
  r.run(10, 0.5f, proto::kFlagArm);
  EXPECT_EQ(r.vehicle.safety().state(), proto::VehicleState::Driving);
}

TEST(CanNode, HeartbeatEchoMeasuresRttOverCan) {
  CanRig r;
  r.op.send_heartbeat();
  r.vehicle.tick();
  r.clock.advance(30ms);
  r.op.poll();
  ASSERT_TRUE(r.op.rtt().has_value());
  EXPECT_EQ(*r.op.rtt(), 30ms);
}
