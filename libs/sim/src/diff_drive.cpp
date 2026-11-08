#include "teledrive/sim/diff_drive.hpp"

#include <algorithm>  // std::clamp
#include <cmath>

namespace teledrive::sim {

namespace {

// 把角度规范到 (-π, π]。
// 不规范会怎样？长时间原地旋转后 heading 会累积到几百 rad，
// 不仅遥测数据难看，float 精度也会随数值变大而变差。
double wrap_angle(double a) {
  a = std::fmod(a, 2.0 * kPi);  // 先落到 (-2π, 2π)
  if (a > kPi) {
    a -= 2.0 * kPi;
  } else if (a <= -kPi) {
    a += 2.0 * kPi;
  }
  return a;
}

// 速度的"变化率限制"（slew rate limiter）：
// 让 current 向 target 靠近，但每一步最多变化 (加速度 * dt)。
//   - 速度绝对值变大：用 accel 限制
//   - 速度绝对值变小，或正在反向：用 decel 限制（刹车更猛）
double slew(double current, double target, double accel, double decel, double dt) {
  const bool reducing = (target * current < 0.0) || (std::abs(target) < std::abs(current));
  const double max_delta = (reducing ? decel : accel) * dt;
  const double delta = std::clamp(target - current, -max_delta, max_delta);
  return current + delta;
}

// 把 float 指令转成 double，NaN/Inf 当成 0。
// 正常情况下 safety 层已经拦截了这类值，这里是"第二道防线"：
// 模型自己不应该因为上游的 bug 而产生 NaN 位置（一旦位置变成 NaN，之后永远是 NaN）。
double finite_or_zero(float v) { return std::isfinite(v) ? static_cast<double>(v) : 0.0; }

}  // namespace

void DiffDriveModel::step(const protocol::Command& target, double dt) {
  // 步长必须是有限正数，否则忽略这次调用
  if (!std::isfinite(dt) || dt <= 0.0) {
    return;
  }

  // 1) 确定目标速度：清洗 NaN → 限制在物理上限内
  double v_target = std::clamp(finite_or_zero(target.linear),
                               -params_.max_linear_speed, params_.max_linear_speed);
  double w_target = std::clamp(finite_or_zero(target.angular),
                               -params_.max_angular_speed, params_.max_angular_speed);

  // 2) 没电了就不再响应指令（目标强制为 0，车会按减速度刹停）
  if (state_.battery_pct <= 0.0) {
    v_target = 0.0;
    w_target = 0.0;
  }

  // 3) 速度向目标靠近（受加速度限制）
  state_.linear_velocity = slew(state_.linear_velocity, v_target,
                                params_.max_linear_accel, params_.max_linear_decel, dt);
  state_.angular_velocity = slew(state_.angular_velocity, w_target,
                                 params_.max_angular_accel, params_.max_angular_decel, dt);

  // 4) 积分位姿（中点法）：用"这一步朝向变化的中间值"推进位置
  const double v = state_.linear_velocity;
  const double w = state_.angular_velocity;
  const double mid_heading = state_.pose.heading + 0.5 * w * dt;
  state_.pose.position.x += v * std::cos(mid_heading) * dt;
  state_.pose.position.y += v * std::sin(mid_heading) * dt;
  state_.pose.heading = wrap_angle(state_.pose.heading + w * dt);

  // 5) 电量：静止耗电 + 与速度成正比的运动耗电；不会低于 0
  const double drain =
      (params_.battery_idle_drain + params_.battery_motion_drain_per_mps * std::abs(v)) * dt;
  state_.battery_pct = std::max(0.0, state_.battery_pct - drain);
}

protocol::Telemetry DiffDriveModel::telemetry(protocol::VehicleState vehicle_state) const {
  protocol::Telemetry t;
  // double → float：协议里是 float32，这里是精度的有意降低
  t.x = static_cast<float>(state_.pose.position.x);
  t.y = static_cast<float>(state_.pose.position.y);
  t.heading = static_cast<float>(state_.pose.heading);
  t.speed = static_cast<float>(state_.linear_velocity);
  t.state = vehicle_state;
  // 电量四舍五入成整数百分比，并夹在 0~100 内再转 uint8
  t.battery_pct = static_cast<std::uint8_t>(std::clamp(std::lround(state_.battery_pct), 0L, 100L));
  return t;
}

}  // namespace teledrive::sim
