#include "teledrive/node/manual_input.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace teledrive::node {

namespace {
// 把非常接近 0 的值吸附成精确的 0。
// 浮点累加 0.1+0.1+0.1-0.1-0.1-0.1 未必恰好等于 0，
// 而安全层判断"速度为零"用的是精确比较。
float snap_to_zero(float v) { return std::fabs(v) < 1e-4f ? 0.0f : v; }
}  // namespace

void ManualInput::nudge(float& value, float delta, float limit) {
  if (!armed_) {
    return;  // 没 arm 时忽略速度键
  }
  // 手动接管：自动模式下按任何行驶键，立刻退出自动。
  // 这是一个安全设计：操作员伸手去碰方向，就说明他想自己控制。
  auto_ = false;
  value = snap_to_zero(std::clamp(value + delta, -limit, limit));
}

void ManualInput::handle_key(char key) {
  switch (std::tolower(static_cast<unsigned char>(key))) {
    case 'e':
      armed_ = !armed_;
      auto_ = false;               // arm 状态一变，自动模式一律退出
      if (!armed_) stop_motion();
      break;
    case 'm':
      if (armed_) {                // 只有 arm 之后才能进入自动
        auto_ = !auto_;
        stop_motion();             // 切换时手动速度清零，避免"退出自动瞬间冲出去"
      }
      break;
    case 'w': nudge(linear_, +config_.linear_step, config_.max_linear); break;
    case 's': nudge(linear_, -config_.linear_step, config_.max_linear); break;
    case 'a': nudge(angular_, +config_.angular_step, config_.max_angular); break;
    case 'd': nudge(angular_, -config_.angular_step, config_.max_angular); break;
    case ' ':
      auto_ = false;
      stop_motion();
      break;
    case 'x':
      estop_left_ = config_.flag_repeat;
      armed_ = false;
      auto_ = false;
      stop_motion();
      break;
    case 'r':
      reset_left_ = config_.flag_repeat;
      armed_ = false;
      auto_ = false;
      stop_motion();
      break;
    case 'q': quit_ = true; break;
    default: break;
  }
}

protocol::Command ManualInput::next_command() {
  protocol::Command c;
  if (estop_left_ > 0) {
    --estop_left_;
    c.flags = protocol::kFlagEStop;
    return c;
  }
  if (reset_left_ > 0) {
    --reset_left_;
    c.flags = protocol::kFlagReset;
    return c;
  }
  c.linear = linear_;
  c.angular = angular_;
  c.flags = armed_ ? protocol::kFlagArm : std::uint8_t{0};
  if (auto_) {
    c.flags |= protocol::kFlagAuto;   // 自动模式下速度字段是 0，速度由车辆端算
  }
  return c;
}

}  // namespace teledrive::node
