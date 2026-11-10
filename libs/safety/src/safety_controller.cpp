#include "teledrive/safety/safety_controller.hpp"

namespace teledrive::safety {

namespace {

using protocol::Command;
using protocol::VehicleState;

// 判断 flags 里某个标志位是否被置位
bool has_flag(std::uint8_t flags, std::uint8_t bit) { return (flags & bit) != 0; }

// 速度指令是否为零。这里用精确比较是有意的：
// "零"来自操作端摇杆死区处理后的精确 0，而不是计算结果。
//bool is_zero_motion(const Command& c) { return c.linear == 0.0f && c.angular == 0.0f; }
bool is_zero_motion(const Command& c) {
  return !has_flag(c.flags, protocol::kFlagAuto) && c.linear == 0.0f && c.angular == 0.0f;
}
}  // namespace

SafetyController::SafetyController(const IClock& clock, SafetyConfig config)
    : config_(config), watchdog_(clock, config.link_timeout) {}

void SafetyController::enter_fault(FaultReason reason) {
  state_ = VehicleState::Fault;
  fault_ = reason;
  target_ = Command{};  // 清掉旧指令，确保不会被意外复用
  watchdog_.stop();     // Fault 下不再需要监视
}

void SafetyController::on_command(const Command& cmd) {
  // ---- 规则 1：急停优先级最高，任何状态都先于其他判断 ----
  if (has_flag(cmd.flags, protocol::kFlagEStop)) {
    enter_fault(FaultReason::EStop);
    return;
  }

  // ---- 规则 2：Fault 锁存，只有复位标志能退出，其余指令一律忽略 ----
  if (state_ == VehicleState::Fault) {
    if (has_flag(cmd.flags, protocol::kFlagReset)) {
      state_ = VehicleState::Idle;
      fault_ = FaultReason::None;
    }
    return;
  }

  // ---- 规则 3：死手开关。arm 必须持续按住，松开就回 Idle ----
  if (!has_flag(cmd.flags, protocol::kFlagArm)) {
    state_ = VehicleState::Idle;
    target_ = Command{};
    watchdog_.stop();
    return;
  }

  // ---- 到这里：arm 按住，状态是 Idle / Armed / Driving 之一 ----
  // 这是一条"来自操作员的有效指令"，喂狗
  watchdog_.feed();

  // ---- 规则 4：数值合法性检查与限幅 ----
  const auto safe = sanitize(cmd, config_.limits);
  if (!safe) {
    enter_fault(FaultReason::InvalidCommand);
    return;
  }

  // ---- 规则 5：正常状态转换 ----
  switch (state_) {
    case VehicleState::Idle:
      // 上电这一周期不执行速度：必须先进入 Armed，下一条指令才可能开始行驶
      state_ = VehicleState::Armed;
      target_ = Command{};
      break;

    case VehicleState::Armed:
      if (!is_zero_motion(*safe)) {
        state_ = VehicleState::Driving;
        target_ = *safe;
      }
      break;

    case VehicleState::Driving:
      if (is_zero_motion(*safe)) {
        state_ = VehicleState::Armed;
        target_ = Command{};
      } else {
        target_ = *safe;
      }
      break;

    case VehicleState::Fault:
      break;  // 上面已处理，这里不会到达；列出来是为了让编译器检查 switch 完整性
  }
}

void SafetyController::update() {
  // 只有"已上电"的状态才需要监视指令是否持续到达
  const bool powered =
      state_ == VehicleState::Armed || state_ == VehicleState::Driving;
  if (powered && watchdog_.expired()) {
    enter_fault(FaultReason::LinkTimeout);
  }
}

Command SafetyController::output() const {
  // fail-safe：只有 Driving 才放行，其他一律零指令
  return state_ == VehicleState::Driving ? target_ : Command{};
}

}  // namespace teledrive::safety
