#include "teledrive/control/pid.hpp"

#include <algorithm>
#include <cmath>

namespace teledrive::control {

double Pid::update(double error, double measurement, double dt) {
  // 防御：dt 或输入不合法时，不改变任何状态，返回上次输出。
  // NaN 一旦进入积分项就永远清不掉，所以必须在入口拦截。
  if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(error) || !std::isfinite(measurement)) {
    return last_output_;
  }

  // ---- 微分项：对测量值求导，取负号（测量值上升 → 负修正，起阻尼作用）----
  double d_term = 0.0;
  if (!first_) {
    const double raw = -(measurement - prev_measurement_) / dt;
    // 一阶低通：alpha=1 时 d_filtered_ = raw（无滤波）
    d_filtered_ += config_.d_filter_alpha * (raw - d_filtered_);
    d_term = config_.kd * d_filtered_;
  }
  prev_measurement_ = measurement;
  first_ = false;

  // ---- 先算"如果这一步积分"的候选值 ----
  const double p_term = config_.kp * error;
  const double candidate_integral = integral_ + error * dt;
  const double unsat = p_term + config_.ki * candidate_integral + d_term;
  const double out = std::clamp(unsat, config_.out_min, config_.out_max);

  // ---- 条件积分（抗饱和）：----
  // 输出没饱和 → 正常积分；
  // 输出饱和，但误差方向能把输出拉回范围内 → 也允许积分（帮助退出饱和）；
  // 输出饱和且误差还在往饱和方向推 → 冻结积分。
  const bool saturated_high = unsat > config_.out_max && error > 0.0;
  const bool saturated_low = unsat < config_.out_min && error < 0.0;
  if (!saturated_high && !saturated_low) {
    integral_ = candidate_integral;
  }

  last_output_ = out;
  return out;
}

void Pid::reset() {
  integral_ = 0.0;
  prev_measurement_ = 0.0;
  d_filtered_ = 0.0;
  last_output_ = 0.0;
  first_ = true;
}

}  // namespace teledrive::control

