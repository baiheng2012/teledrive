#include "teledrive/safety/limits.hpp"

#include <algorithm>  // std::clamp（C++17）
#include <cmath>      // std::isfinite

namespace teledrive::safety {

std::optional<protocol::Command> sanitize(const protocol::Command& in,
                                          const SafetyLimits& limits) {
  // std::isfinite：既不是 NaN 也不是 ±Inf 才返回 true。
  // 必须先检查再 clamp：clamp 对 NaN 的行为没有意义（NaN 与任何数比较都为 false）。
  if (!std::isfinite(in.linear) || !std::isfinite(in.angular)) {
    return std::nullopt;
  }

  protocol::Command out = in;  // 拷贝一份，flags 等其他字段原样保留
  out.linear = std::clamp(in.linear, -limits.max_linear, limits.max_linear);
  out.angular = std::clamp(in.angular, -limits.max_angular, limits.max_angular);
  return out;
}

}  // namespace teledrive::safety
