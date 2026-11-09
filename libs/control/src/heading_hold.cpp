#include "teledrive/control/heading_hold.hpp"

#include <cmath>

namespace teledrive::control {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double angle_diff(double target, double current) {
  double d = std::fmod(target - current, 2.0 * kPi);
  if (d > kPi) {
    d -= 2.0 * kPi;
  } else if (d <= -kPi) {
    d += 2.0 * kPi;
  }
  return d;
}

double HeadingHold::update(double operator_angular, double heading, double linear_speed,
                           double dt) {
  const bool operator_steering = std::abs(operator_angular) > config_.steer_deadband;
  const bool moving = std::abs(linear_speed) >= config_.min_speed;

  // 操作员在打方向，或车基本没动：不介入，并放弃锁定（下次直行时重新锁定当前朝向）
  if (operator_steering || !moving) {
    holding_ = false;
    pid_.reset();  // 清掉旧状态，避免下次介入时带着过期的积分
    return operator_angular;
  }

  // 刚进入保持状态：把"此刻的朝向"记为目标
  if (!holding_) {
    holding_ = true;
    target_ = heading;
    pid_.reset();
  }

  // 误差用回绕后的最短角度差；微分用测量值（朝向）
  const double error = angle_diff(target_, heading);
  return operator_angular + pid_.update(error, heading, dt);
}

}  // namespace teledrive::control
