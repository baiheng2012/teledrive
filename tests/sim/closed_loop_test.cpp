// 闭环测试：SafetyController（守门人）+ DiffDriveModel（车）+ FakeClock（假时间）。
// 用 50Hz 的控制循环模拟真实运行，整个过程不依赖真实时间，瞬间完成且完全可重复。
#include <gtest/gtest.h>
#include <teledrive/safety/safety_controller.hpp>
#include <teledrive/sim/diff_drive.hpp>

using namespace std::chrono_literals;
using teledrive::FakeClock;
using teledrive::protocol::Command;
using teledrive::protocol::VehicleState;
using teledrive::protocol::kFlagArm;
using teledrive::protocol::kFlagEStop;
using teledrive::safety::FaultReason;
using teledrive::safety::SafetyController;
using teledrive::sim::DiffDriveModel;

namespace {

constexpr double kDt = 0.02;  // 50Hz

class ClosedLoop : public ::testing::Test {
 protected:
  FakeClock clock;
  SafetyController safety{clock};
  DiffDriveModel vehicle;

  // 一个控制周期：（可选）收到指令 → 看门狗检查 → 把安全输出交给车 → 时间前进 20ms
  // 这个顺序就是第 6 天 vehicle_sim 主循环的雏形。
  void tick(const Command* incoming) {
    if (incoming != nullptr) {
      safety.on_command(*incoming);
    }
    safety.update();
    vehicle.step(safety.output(), kDt);
    clock.advance(20ms);
  }

  // 持续下达"arm + 1 m/s"指令 n 个周期
  void drive(int ticks) {
    Command c;
    c.linear = 1.0f;
    c.flags = kFlagArm;
    for (int i = 0; i < ticks; ++i) {
      tick(&c);
    }
  }

  void coast_without_commands(int ticks) {
    for (int i = 0; i < ticks; ++i) {
      tick(nullptr);
    }
  }
};

}  // namespace

TEST_F(ClosedLoop, DrivesToCruiseSpeedUnderOperatorControl) {
  drive(200);  // 4 秒
  EXPECT_EQ(safety.state(), VehicleState::Driving);
  EXPECT_NEAR(vehicle.state().linear_velocity, 1.0, 1e-6);
  EXPECT_GT(vehicle.state().pose.position.x, 2.0);  // 确实开出去了
}

// 失联：操作员的指令突然全部停止。
//   前 200ms：看门狗还没超时，车按最后一条指令继续跑（约 0.2 m）
//   超时后：进入 Fault，目标速度归零，车以 2 m/s² 刹停（约 0.25 m）
// 所以"失联后总滑行距离"应当在 0.5 m 左右。这个数字就是安全设计的量化指标。
TEST_F(ClosedLoop, LinkLossStopsTheVehicleWithinBoundedDistance) {
  drive(200);
  ASSERT_NEAR(vehicle.state().linear_velocity, 1.0, 1e-6);
  const double x_at_loss = vehicle.state().pose.position.x;

  coast_without_commands(100);  // 失联 2 秒

  EXPECT_EQ(safety.state(), VehicleState::Fault);
  EXPECT_EQ(safety.fault_reason(), FaultReason::LinkTimeout);
  EXPECT_NEAR(vehicle.state().linear_velocity, 0.0, 1e-9);  // 最终完全停下

  const double coasted = vehicle.state().pose.position.x - x_at_loss;
  EXPECT_GT(coasted, 0.3);   // 不可能瞬间停住：有看门狗等待 + 刹车距离
  EXPECT_LT(coasted, 0.6);   // 但必须在设计范围内
}

// 急停：下一个周期车就开始以最大减速度刹车，不需要等任何超时
TEST_F(ClosedLoop, EStopBrakesImmediately) {
  drive(200);
  const double x_before = vehicle.state().pose.position.x;

  Command estop;
  estop.linear = 1.0f;  // 即使指令里还带着速度，急停也必须压过它
  estop.flags = kFlagArm | kFlagEStop;
  tick(&estop);
  EXPECT_EQ(safety.state(), VehicleState::Fault);
  EXPECT_EQ(safety.fault_reason(), FaultReason::EStop);

  coast_without_commands(50);  // 1 秒后
  EXPECT_NEAR(vehicle.state().linear_velocity, 0.0, 1e-9);
  EXPECT_LT(vehicle.state().pose.position.x - x_before, 0.35);  // 刹车距离约 0.25 m
}
