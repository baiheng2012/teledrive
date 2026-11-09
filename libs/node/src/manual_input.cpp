#include "teledrive/node/manual_input.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace teledrive::node {

namespace {
// 把非常接近 0 的值吸附成精确的 0。
// 原因：浮点累加 0.1+0.1+0.1-0.1-0.1-0.1 未必恰好等于 0，
// 而安全层判断"速度为零"用的是精确比较（Driving → Armed 的条件）。
float snap_to_zero(float v) { return std::fabs(v) < 1e-4f ? 0.0f : v; }
}  // namespace

void ManualInput::nudge(float& value, float delta, float limit) {
  if (!armed_) {
    return;  // 没 arm 时忽略速度键，避免"先按 w 再 arm"带来的意外起步
  }
  value = snap_to_zero(std::clamp(value + delta, -limit, limit));
}

void ManualInput::handle_key(char key) {
  // tolower 要求参数是 unsigned char 范围内的值，先转换再调用
  switch (std::tolower(static_cast<unsigned char>(key))) {
    case 'e':
      armed_ = !armed_;
      if (!armed_) stop_motion();  // 解除 arm 时速度一并清零
      break;
    case 'w': nudge(linear_, +config_.linear_step, config_.max_linear); break;
    case 's': nudge(linear_, -config_.linear_step, config_.max_linear); break;
    case 'a': nudge(angular_, +config_.angular_step, config_.max_angular); break;
    case 'd': nudge(angular_, -config_.angular_step, config_.max_angular); break;
    case ' ': stop_motion(); break;
    case 'x':
      estop_left_ = config_.flag_repeat;
      armed_ = false;
      stop_motion();
      break;
    case 'r':
      reset_left_ = config_.flag_repeat;
      armed_ = false;
      stop_motion();
      break;
    case 'q': quit_ = true; break;
    default: break;
  }
}

protocol::Command ManualInput::next_command() {
  protocol::Command c;
  // 急停和复位期间只发对应标志，速度为零。急停优先于复位。
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
  return c;
}

}  // namespace teledrive::node
