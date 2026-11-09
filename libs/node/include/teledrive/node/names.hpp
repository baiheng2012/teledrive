#pragma once
// 把枚举转成可读的字符串，用于终端显示和日志。
// 用 switch 而不加 default：如果以后给枚举新增了值而忘了这里，
// 编译器会通过 -Wswitch 警告，提醒补上。

#include "teledrive/protocol/messages.hpp"
#include "teledrive/safety/safety_controller.hpp"

namespace teledrive::node {

inline const char* name(protocol::VehicleState s) {
  switch (s) {
    case protocol::VehicleState::Idle:    return "IDLE";
    case protocol::VehicleState::Armed:   return "ARMED";
    case protocol::VehicleState::Driving: return "DRIVING";
    case protocol::VehicleState::Fault:   return "FAULT";
  }
  return "?";  // 收到了协议里不存在的状态值（比如被篡改但 CRC 碰巧对）
}

inline const char* name(safety::FaultReason r) {
  switch (r) {
    case safety::FaultReason::None:           return "-";
    case safety::FaultReason::EStop:          return "ESTOP";
    case safety::FaultReason::LinkTimeout:    return "LINK_TIMEOUT";
    case safety::FaultReason::InvalidCommand: return "INVALID_CMD";
  }
  return "?";
}

}  // namespace teledrive::node
