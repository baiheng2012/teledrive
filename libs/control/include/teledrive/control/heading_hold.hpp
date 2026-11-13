#pragma once
// 航向保持：操作员不打方向时，把车锁定在当前朝向。
// 这是最简单的一种"共享控制"：操作员的转向指令优先，辅助只在操作员"没有意见"时介入。

//#include "teledrive/control/pid.hpp"
#include <memory>
#include "teledrive/control/pid_core.hpp"

namespace teledrive::control {

struct HeadingHoldConfig {
  PidConfig pid{2.0, 0.2, 0.1, -1.0, 1.0, 0.5};  // kp ki kd out_min out_max alpha
  double steer_deadband{0.02};     // rad/s：小于它视为"操作员没打方向"
  double min_speed{0.05};          // m/s：低于它不做修正
};

class HeadingHold {
 public:
  //explicit HeadingHold(HeadingHoldConfig config = {}) : config_(config), pid_(config.pid) {}
  explicit HeadingHold(HeadingHoldConfig config = {},
                     std::unique_ptr<IPidCore> pid = nullptr);

  // 输入：操作员的角速度指令、当前朝向、当前线速度、步长
  // 输出：实际应该下发的角速度
  double update(double operator_angular, double heading, double linear_speed, double dt);

  bool holding() const { return holding_; }
  double target_heading() const { return target_; }

 private:
  HeadingHoldConfig config_;
  //Pid pid_;
  std::unique_ptr<IPidCore> pid_;
  bool holding_{false};
  double target_{0.0};
};

// 角度差规范到 (-π, π]：target - current，走最短的方向
double angle_diff(double target, double current);

}  // namespace teledrive::control
