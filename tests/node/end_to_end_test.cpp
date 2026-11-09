// 端到端测试：OperatorSession ⇄ MockTransport ⇄ VehicleNode，全部在内存里，
// 时间由 FakeClock 控制。没有真实 socket、没有 sleep，整个文件几毫秒跑完。
#include <gtest/gtest.h>
#include <teledrive/node/operator_session.hpp>
#include <teledrive/node/vehicle_node.hpp>
#include <teledrive/transport/mock_transport.hpp>

using namespace std::chrono_literals;
using teledrive::FakeClock;
using teledrive::node::OperatorSession;
using teledrive::node::VehicleNode;
using teledrive::safety::FaultReason;
using teledrive::transport::Endpoint;
using teledrive::transport::MockTransport;
namespace proto = teledrive::protocol;
using proto::VehicleState;

namespace {

proto::Command make_cmd(float lin, float ang, std::uint8_t flags) {
  proto::Command c;
  c.linear = lin;
  c.angular = ang;
  c.flags = flags;
  return c;
}

const Endpoint kOp{0x7F000001u, 9000};
const Endpoint kVeh{0x7F000001u, 9001};

class EndToEnd : public ::testing::Test {
 protected:
  // 成员按声明顺序初始化：clock 和两个 transport 必须先于使用它们的对象
  FakeClock clock;
  MockTransport op_t{kOp};
  MockTransport veh_t{kVeh};
  VehicleNode vehicle{veh_t, clock};
  OperatorSession op{op_t, clock, kVeh};

  void SetUp() override {
    op_t.connect(&veh_t);   // 操作端 → 车辆端
    veh_t.connect(&op_t);   // 车辆端 → 操作端（遥测）
  }

  // 一个 50Hz 周期：车辆处理 → 操作端收取 → 时间前进 20ms
  void step() {
    vehicle.tick();
    op.poll();
    clock.advance(20ms);
  }

  // 每个周期都发 cmd（可为空表示"不发"），共 n 个周期
  void run(int n, const proto::Command* cmd) {
    for (int i = 0; i < n; ++i) {
      if (cmd) op.send_command(*cmd);
      step();
    }
  }

  // 把车带到稳定行驶状态
  void drive_to_cruise() {
    const auto arm = make_cmd(0, 0, proto::kFlagArm);
    const auto go = make_cmd(0.5f, 0, proto::kFlagArm);
    run(2, &arm);
    run(50, &go);
    ASSERT_EQ(vehicle.safety().state(), VehicleState::Driving);
  }

  // 绕过 OperatorSession，直接往车辆端注入一个指定 seq 的指令（用来构造乱序、重放）
  void inject(std::uint16_t seq, const proto::Command& c) {
    const auto e = proto::encode(seq, c);
    veh_t.inject(kOp, e.data.data(), e.size);
  }
};

}  // namespace

// ---------- 正常流程 ----------

TEST_F(EndToEnd, OperatorCanArmAndDriveVehicle) {
  const auto arm = make_cmd(0, 0, proto::kFlagArm);
  const auto go = make_cmd(0.5f, 0, proto::kFlagArm);

  run(2, &arm);
  EXPECT_EQ(vehicle.safety().state(), VehicleState::Armed);

  run(50, &go);  // 1 秒，加速到 0.5 m/s 需要 0.5 秒
  EXPECT_EQ(vehicle.safety().state(), VehicleState::Driving);
  EXPECT_NEAR(vehicle.model().state().linear_velocity, 0.5, 1e-6);
  EXPECT_GT(vehicle.model().state().pose.position.x, 0.2);
}

TEST_F(EndToEnd, TelemetryReachesOperator) {
  EXPECT_FALSE(op.latest_telemetry().has_value());  // 还没通信过
  EXPECT_FALSE(op.link_alive(500ms));

  drive_to_cruise();

  ASSERT_TRUE(op.latest_telemetry().has_value());
  EXPECT_EQ(op.latest_telemetry()->state, VehicleState::Driving);
  EXPECT_NEAR(op.latest_telemetry()->speed, 0.5f, 1e-3f);
  EXPECT_TRUE(op.link_alive(500ms));

  clock.advance(1s);      // 车辆端不再发送
  EXPECT_FALSE(op.link_alive(500ms));
}

TEST_F(EndToEnd, HeartbeatEchoMeasuresRtt) {
  op.send_heartbeat();
  vehicle.tick();          // 车辆收到后立刻回显
  clock.advance(40ms);     // 假设往返花了 40ms
  op.poll();
  ASSERT_TRUE(op.rtt().has_value());
  EXPECT_EQ(*op.rtt(), 40ms);
}

TEST_F(EndToEnd, EStopFromOperatorFaultsVehicle) {
  drive_to_cruise();
  op.send_command(make_cmd(0.5f, 0, proto::kFlagArm | proto::kFlagEStop));
  step();
  EXPECT_EQ(vehicle.safety().state(), VehicleState::Fault);
  EXPECT_EQ(vehicle.safety().fault_reason(), FaultReason::EStop);
}

// ---------- 故障场景 ----------

TEST_F(EndToEnd, CommandLossFaultsVehicleAndOperatorSeesIt) {
  drive_to_cruise();
  run(25, nullptr);  // 操作端沉默 0.5 秒（超过 200ms 超时）

  EXPECT_EQ(vehicle.safety().state(), VehicleState::Fault);
  EXPECT_EQ(vehicle.safety().fault_reason(), FaultReason::LinkTimeout);
  ASSERT_TRUE(op.latest_telemetry().has_value());
  EXPECT_EQ(op.latest_telemetry()->state, VehicleState::Fault);  // 遥测把故障告诉了操作员
}

// 这条测试把第 4 天"看门狗只由指令喂"的设计决策落到整个系统上：
// 链路活着、心跳不断，但指令停了 —— 车必须停。
TEST_F(EndToEnd, HeartbeatsAloneDoNotKeepVehicleAlive) {
  drive_to_cruise();
  for (int i = 0; i < 25; ++i) {
    op.send_heartbeat();   // 只发心跳，不发指令
    step();
  }
  EXPECT_EQ(vehicle.safety().state(), VehicleState::Fault);
  EXPECT_EQ(vehicle.safety().fault_reason(), FaultReason::LinkTimeout);
}

// 50Hz 发送，连丢 3 个包（60ms）远小于 200ms 超时，车不应受影响
TEST_F(EndToEnd, ShortPacketLossIsTolerated) {
  drive_to_cruise();
  const auto go = make_cmd(0.5f, 0, proto::kFlagArm);
  op_t.drop_next_sends(3);
  run(8, &go);
  EXPECT_EQ(vehicle.safety().state(), VehicleState::Driving);
}

// ---------- 坏包与序列号 ----------

TEST_F(EndToEnd, CorruptedPacketIsCountedAndIgnored) {
  const std::uint8_t junk[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  veh_t.inject(kOp, junk, sizeof(junk));
  vehicle.tick();
  EXPECT_EQ(vehicle.stats().rx_bad, 1u);
  EXPECT_EQ(vehicle.stats().rx_ok, 0u);
  EXPECT_EQ(vehicle.safety().state(), VehicleState::Idle);
}

TEST_F(EndToEnd, StaleSequenceNumbersAreRejected) {
  const auto arm = make_cmd(0, 0, proto::kFlagArm);
  const auto estop = make_cmd(0, 0, proto::kFlagEStop);

  inject(10, arm);
  vehicle.tick();
  ASSERT_EQ(vehicle.safety().state(), VehicleState::Armed);

  inject(5, estop);  // 旧包（可能是延迟到达或被重放的），必须忽略
  vehicle.tick();
  EXPECT_EQ(vehicle.safety().state(), VehicleState::Armed);
  EXPECT_EQ(vehicle.stats().rx_stale, 1u);

  inject(11, estop);  // 新包，生效
  vehicle.tick();
  EXPECT_EQ(vehicle.safety().state(), VehicleState::Fault);
}

TEST_F(EndToEnd, DuplicateSequenceIsRejected) {
  const auto arm = make_cmd(0, 0, proto::kFlagArm);
  inject(7, arm);
  inject(7, arm);
  vehicle.tick();
  EXPECT_EQ(vehicle.stats().rx_ok, 1u);
  EXPECT_EQ(vehicle.stats().rx_stale, 1u);
}

TEST_F(EndToEnd, SequenceWrapAroundIsAccepted) {
  const auto arm = make_cmd(0, 0, proto::kFlagArm);
  inject(65535, arm);
  vehicle.tick();
  inject(0, arm);  // 65535 的下一个就是 0
  vehicle.tick();
  EXPECT_EQ(vehicle.stats().rx_ok, 2u);
  EXPECT_EQ(vehicle.stats().rx_stale, 0u);
}

// 操作端重启后 seq 从头开始：静默超过 1 秒，车辆端重新同步，不会永远拒绝
TEST_F(EndToEnd, ResyncsAfterLongSilence) {
  const auto arm = make_cmd(0, 0, proto::kFlagArm);
  inject(1000, arm);
  vehicle.tick();

  clock.advance(2s);
  inject(3, arm);  // seq 远小于 1000，但已经静默很久
  vehicle.tick();
  EXPECT_EQ(vehicle.stats().rx_ok, 2u);
  EXPECT_EQ(vehicle.stats().rx_stale, 0u);
}
