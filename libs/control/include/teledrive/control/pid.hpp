#pragma once
// PID 控制器。
//   - 抗积分饱和：条件积分（输出饱和且误差仍推向饱和方向时，不累积积分）
//   - 微分作用在测量值上（避免目标突变时的 derivative kick）
//   - 微分一阶低通滤波（抑制测量噪声）
// 纯计算类：不读时钟，dt 由调用者传入，因此测试完全确定。

namespace teledrive::control {

struct PidConfig {
  double kp{1.0};
  double ki{0.0};
  double kd{0.0};
  double out_min{-1.0};          // 输出下限
  double out_max{1.0};           // 输出上限
  double d_filter_alpha{1.0};    // 微分滤波系数 (0,1]：1 = 不滤波，越小越平滑
};

class Pid {
 public:
  explicit Pid(PidConfig config = {}) : config_(config) {}

  // 计算一步。
  //   error       ：目标 − 实际（调用者算好，角度要先做回绕处理）
  //   measurement ：当前测量值，仅用于微分项
  //   dt          ：步长（秒），非有限或非正则返回上一次输出，状态不变
  double update(double error, double measurement, double dt);

  // 清空积分和微分历史（比如切换模式时，避免带着旧状态）
  void reset();

  double integral() const { return integral_; }

 private:
  PidConfig config_;
  double integral_{0.0};
  double prev_measurement_{0.0};
  double d_filtered_{0.0};
  double last_output_{0.0};
  bool first_{true};
};

}  // namespace teledrive::control
