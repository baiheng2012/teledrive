#pragma once
// 指令限幅与合法性检查。
// 两件事：
//   1) 数值不可信（NaN、无穷大）-> 拒绝，调用者应当进入故障状态
//   2) 数值可信但超范围 -> 夹到上限（clamp），而不是拒绝
// 区别对待的原因：超范围通常只是操作员推满了摇杆，限幅即可；
// 而 NaN/Inf 说明上游代码或数据已经出错，继续相信这条指令是危险的。

#include <optional>

#include "teledrive/protocol/messages.hpp"

namespace teledrive::safety {

struct SafetyLimits {
  float max_linear{1.0f};   // 最大线速度 m/s（正反方向对称）
  float max_angular{1.5f};  // 最大角速度 rad/s
};

// 返回限幅后的指令（flags 原样保留）；指令含 NaN/Inf 时返回 std::nullopt。
std::optional<protocol::Command> sanitize(const protocol::Command& in,
                                          const SafetyLimits& limits);

}  // namespace teledrive::safety
