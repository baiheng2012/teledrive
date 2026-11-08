#include <gtest/gtest.h>
#include <teledrive/safety/safety_controller.hpp>

#include <limits>

using namespace std::chrono_literals;
using teledrive::FakeClock;
using teledrive::protocol::Command;
using teledrive::protocol::VehicleState;
using teledrive::protocol::kFlagArm;
using teledrive::protocol::kFlagEStop;
using teledrive::protocol::kFlagReset;
using teledrive::safety::FaultReason;
using teledrive::safety::SafetyController;

namespace {

Command make(float lin, float ang, std::uint8_t flags) {
  Command c;
  c.linear = lin;
  c.angular = ang;
  c.flags = flags;
  return c;
}

// 辅助：把控制器带到指定状态，作为表驱动测试的"起点"
void reach(SafetyController& c, VehicleState target) {
  switch (target) {
    case VehicleState::Idle:
      break;
    case VehicleState::Armed:
      c.on_command(make(0, 0, kFlagArm));
      break;
    case VehicleState::Driving:
      c.on_command(make(0, 0, kFlagArm));
      c.on_command(make(0.5f, 0, kFlagArm));
      break;
    case VehicleState::Fault:
      c.on_command(make(0, 0, kFlagEStop));
      break;
  }
  ASSERT_EQ(c.state(), target);  // 确认起点正确，否则后面的断言没有意义
}

}  // namespace

// ============ 表驱动：状态转换表 ============
// 每一行 = "从某状态出发，收到某条指令，应当到达什么状态"。
// 想增加规则，只要加一行，不用写新测试。

struct Transition {
  VehicleState from;
  std::uint8_t flags;
  float linear;
  VehicleState expected;
  const char* what;  // 失败时显示，方便定位是哪一行
};

TEST(SafetyTransitions, TableDriven) {
  using S = VehicleState;
  const Transition table[] = {
      // from        flags                   linear  expected      说明
      {S::Idle,    0,                         0.0f,  S::Idle,    "Idle: 无 arm，保持 Idle"},
      {S::Idle,    kFlagArm,                  0.0f,  S::Armed,   "Idle: arm -> Armed"},
      {S::Idle,    kFlagArm,                  0.5f,  S::Armed,   "Idle: arm 周期不开始行驶"},
      {S::Idle,    kFlagEStop,                0.0f,  S::Fault,   "Idle: 急停 -> Fault"},

      {S::Armed,   kFlagArm,                  0.0f,  S::Armed,   "Armed: 零速度保持 Armed"},
      {S::Armed,   kFlagArm,                  0.5f,  S::Driving, "Armed: 有速度 -> Driving"},
      {S::Armed,   0,                         0.0f,  S::Idle,    "Armed: 松开 arm -> Idle"},
      {S::Armed,   kFlagArm | kFlagEStop,     0.0f,  S::Fault,   "Armed: 急停优先于 arm"},

      {S::Driving, kFlagArm,                  0.5f,  S::Driving, "Driving: 持续行驶"},
      {S::Driving, kFlagArm,                  0.0f,  S::Armed,   "Driving: 速度归零 -> Armed"},
      {S::Driving, 0,                         0.5f,  S::Idle,    "Driving: 松开 arm -> Idle（死手开关）"},
      {S::Driving, kFlagEStop,                0.5f,  S::Fault,   "Driving: 急停 -> Fault"},

      {S::Fault,   kFlagArm,                  0.5f,  S::Fault,   "Fault: 锁存，忽略 arm 和速度"},
      {S::Fault,   0,                         0.0f,  S::Fault,   "Fault: 无复位标志，保持"},
      {S::Fault,   kFlagReset,                0.0f,  S::Idle,    "Fault: 复位 -> Idle"},
      {S::Fault,   kFlagReset | kFlagEStop,   0.0f,  S::Fault,   "Fault: 急停仍按下，复位无效"},
      {S::Fault,   kFlagReset | kFlagArm,     0.5f,  S::Idle,    "Fault: 复位成功但不会同时行驶"},
  };

  for (const auto& row : table) {
    SCOPED_TRACE(row.what);  // 断言失败时，会附带打印这一行的说明
    FakeClock clock;
    SafetyController ctrl(clock);
    reach(ctrl, row.from);
    ctrl.on_command(make(row.linear, 0.0f, row.flags));
    EXPECT_EQ(ctrl.state(), row.expected);
  }
}

// ============ 场景测试：时间、输出、故障原因 ============

class SafetyScenario : public ::testing::Test {
 protected:
  FakeClock clock;
  SafetyController ctrl{clock};  // 默认配置：线速度 ±1.0，角速度 ±1.5，超时 200ms
};

TEST_F(SafetyScenario, StartsIdleWithZeroOutput) {
  EXPECT_EQ(ctrl.state(), VehicleState::Idle);
  EXPECT_EQ(ctrl.fault_reason(), FaultReason::None);
  EXPECT_FLOAT_EQ(ctrl.output().linear, 0.0f);
}

TEST_F(SafetyScenario, OutputIsZeroUntilDriving) {
  ctrl.on_command(make(0.5f, 0.2f, kFlagArm));  // 上电周期，进入 Armed
  EXPECT_EQ(ctrl.state(), VehicleState::Armed);
  EXPECT_FLOAT_EQ(ctrl.output().linear, 0.0f);   // Armed 不放行速度
}

TEST_F(SafetyScenario, DrivingOutputIsPassedThrough) {
  ctrl.on_command(make(0, 0, kFlagArm));
  ctrl.on_command(make(0.5f, -0.3f, kFlagArm));
  EXPECT_EQ(ctrl.state(), VehicleState::Driving);
  EXPECT_FLOAT_EQ(ctrl.output().linear, 0.5f);
  EXPECT_FLOAT_EQ(ctrl.output().angular, -0.3f);
}

TEST_F(SafetyScenario, DrivingOutputIsClampedToLimits) {
  ctrl.on_command(make(0, 0, kFlagArm));
  ctrl.on_command(make(10.0f, -10.0f, kFlagArm));  // 远超上限
  EXPECT_EQ(ctrl.state(), VehicleState::Driving);  // 超范围不是故障，只是限幅
  EXPECT_FLOAT_EQ(ctrl.output().linear, 1.0f);
  EXPECT_FLOAT_EQ(ctrl.output().angular, -1.5f);
}

TEST_F(SafetyScenario, ReleasingArmStopsTheVehicle) {
  ctrl.on_command(make(0, 0, kFlagArm));
  ctrl.on_command(make(0.5f, 0, kFlagArm));
  ctrl.on_command(make(0.5f, 0, 0));  // 松开 arm，速度字段还是 0.5
  EXPECT_EQ(ctrl.state(), VehicleState::Idle);
  EXPECT_FLOAT_EQ(ctrl.output().linear, 0.0f);
}

// 看门狗边界：200ms 内没事，201ms 进入 Fault 并停车
TEST_F(SafetyScenario, LinkTimeoutCausesFaultAndStops) {
  ctrl.on_command(make(0, 0, kFlagArm));
  ctrl.on_command(make(0.5f, 0, kFlagArm));

  clock.advance(200ms);
  ctrl.update();
  EXPECT_EQ(ctrl.state(), VehicleState::Driving);  // 恰好 200ms：还没超时

  clock.advance(1ms);
  ctrl.update();
  EXPECT_EQ(ctrl.state(), VehicleState::Fault);
  EXPECT_EQ(ctrl.fault_reason(), FaultReason::LinkTimeout);
  EXPECT_FLOAT_EQ(ctrl.output().linear, 0.0f);     // 立刻停车
}

TEST_F(SafetyScenario, SteadyCommandsKeepTheLinkAlive) {
  ctrl.on_command(make(0, 0, kFlagArm));
  // 模拟 50Hz（每 20ms 一条指令）持续 4 秒
  for (int i = 0; i < 200; ++i) {
    ctrl.on_command(make(0.5f, 0, kFlagArm));
    clock.advance(20ms);
    ctrl.update();
    ASSERT_EQ(ctrl.state(), VehicleState::Driving) << "第 " << i << " 个周期掉出了 Driving";
  }
}

TEST_F(SafetyScenario, IdleDoesNotTimeOut) {
  clock.advance(10s);  // 操作员一直没来
  ctrl.update();
  EXPECT_EQ(ctrl.state(), VehicleState::Idle);  // 没上电就不监视
}

TEST_F(SafetyScenario, NaNCommandCausesFault) {
  ctrl.on_command(make(0, 0, kFlagArm));
  ctrl.on_command(make(std::numeric_limits<float>::quiet_NaN(), 0, kFlagArm));
  EXPECT_EQ(ctrl.state(), VehicleState::Fault);
  EXPECT_EQ(ctrl.fault_reason(), FaultReason::InvalidCommand);
}

TEST_F(SafetyScenario, EStopRecordsReason) {
  ctrl.on_command(make(0, 0, kFlagArm));
  ctrl.on_command(make(0.5f, 0, kFlagArm));
  ctrl.on_command(make(0.5f, 0, kFlagArm | kFlagEStop));
  EXPECT_EQ(ctrl.state(), VehicleState::Fault);
  EXPECT_EQ(ctrl.fault_reason(), FaultReason::EStop);
  EXPECT_FLOAT_EQ(ctrl.output().linear, 0.0f);
}

// 故障锁存：超时进入 Fault 后，即使指令恢复到达，车辆也不会自己重新动起来
TEST_F(SafetyScenario, FaultIsLatchedAfterTimeoutEvenIfCommandsResume) {
  ctrl.on_command(make(0, 0, kFlagArm));
  ctrl.on_command(make(0.5f, 0, kFlagArm));
  clock.advance(500ms);
  ctrl.update();
  ASSERT_EQ(ctrl.state(), VehicleState::Fault);

  ctrl.on_command(make(0.5f, 0, kFlagArm));  // 网络恢复，指令又来了
  EXPECT_EQ(ctrl.state(), VehicleState::Fault);
  EXPECT_FLOAT_EQ(ctrl.output().linear, 0.0f);
}

TEST_F(SafetyScenario, CanRecoverOnlyViaResetThenReArm) {
  ctrl.on_command(make(0, 0, kFlagEStop));
  ASSERT_EQ(ctrl.state(), VehicleState::Fault);

  ctrl.on_command(make(0, 0, kFlagReset));  // 复位 -> Idle
  EXPECT_EQ(ctrl.state(), VehicleState::Idle);
  EXPECT_EQ(ctrl.fault_reason(), FaultReason::None);

  ctrl.on_command(make(0, 0, kFlagArm));    // 重新 arm
  ctrl.on_command(make(0.5f, 0, kFlagArm)); // 重新行驶
  EXPECT_EQ(ctrl.state(), VehicleState::Driving);
}
