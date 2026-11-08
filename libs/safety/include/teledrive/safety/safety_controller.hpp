#pragma once
// 安全控制器：车辆端所有指令的"守门人"。
//
// 使用方式（车辆端控制循环，50Hz）：
//   每收到一条合法的 Command  -> controller.on_command(cmd)
//   每个控制周期              -> controller.update()          （检查看门狗）
//   然后                      -> auto safe = controller.output() （交给后面的控制算法）
//
// 注意：这里不处理心跳。看门狗只由"有效指令"喂，
// 原因：危险的是"链路活着但指令停了"，心跳不能掩盖这种情况。

#include <cstdint>

#include "teledrive/common/clock.hpp"
#include "teledrive/protocol/messages.hpp"
#include "teledrive/safety/limits.hpp"
#include "teledrive/safety/watchdog.hpp"

namespace teledrive::safety {

// 进入 Fault 的原因，写进日志、显示给操作员，便于排查
enum class FaultReason : std::uint8_t {
  None = 0,
  EStop,           // 操作员触发急停
  LinkTimeout,     // 看门狗超时（失联或指令停止）
  InvalidCommand,  // 指令含 NaN/Inf
};

struct SafetyConfig {
  SafetyLimits limits{};
  Duration link_timeout{std::chrono::milliseconds(200)};  // 指令超时时间
};

class SafetyController {
 public:
  explicit SafetyController(const IClock& clock, SafetyConfig config = {});

  // 处理一条已通过协议层校验的指令
  void on_command(const protocol::Command& cmd);

  // 每个控制周期调用一次：检查看门狗是否超时
  void update();

  protocol::VehicleState state() const { return state_; }
  FaultReason fault_reason() const { return fault_; }

  // 当前允许执行的指令。只有 Driving 状态才非零，其余状态一律返回零指令（fail-safe）
  protocol::Command output() const;

 private:
  void enter_fault(FaultReason reason);

  SafetyConfig config_;
  Watchdog watchdog_;
  protocol::VehicleState state_{protocol::VehicleState::Idle};
  FaultReason fault_{FaultReason::None};
  protocol::Command target_{};  // 最近一条经过限幅的有效指令
};

}  // namespace teledrive::safety
