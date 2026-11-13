#pragma once
// PID 核心的抽象接口。手写 Pid 和 Simulink 生成的 PID 都实现它，
// 上层（HeadingHold）只依赖接口，两种实现可以互换并用同一套测试验证。
#include "teledrive/control/pid.hpp"

namespace teledrive::control {

class IPidCore {
 public:
  virtual ~IPidCore() = default;
  virtual double update(double error, double measurement, double dt) = 0;
  virtual void reset() = 0;
};

// 手写实现：直接包装 Pid
class HandwrittenPid final : public IPidCore {
 public:
  explicit HandwrittenPid(PidConfig c = {}) : pid_(c) {}
  double update(double e, double m, double dt) override { return pid_.update(e, m, dt); }
  void reset() override { pid_.reset(); }

 private:
  Pid pid_;
};

// Simulink / Embedded Coder 生成的实现（见 generated_pid.cpp）
class GeneratedPid final : public IPidCore {
 public:
  explicit GeneratedPid(PidConfig c = {});
  ~GeneratedPid() override;
  GeneratedPid(const GeneratedPid&) = delete;
  GeneratedPid& operator=(const GeneratedPid&) = delete;
  double update(double e, double m, double dt) override;
  void reset() override;

 private:
  struct Impl;          // 把生成代码的头文件隔离在 .cpp 里，不泄漏给使用者
  Impl* impl_;
  PidConfig cfg_;
};

}  // namespace teledrive::control
