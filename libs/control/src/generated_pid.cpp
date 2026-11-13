#include <cmath>
#include <memory>

#include "teledrive/control/pid_core.hpp"
#include "pid_ctrl.h"   // Embedded Coder 生成

namespace teledrive::control {

struct GeneratedPid::Impl {
  // 用 unique_ptr 持有生成的模型对象，这样 reset() 可以销毁并重新构造它。
  // 生成类的初始状态(integ/prev/dfilt/first)是在构造函数里设置的。
  std::unique_ptr<pid_ctrl> model = std::make_unique<pid_ctrl>();
  double last_output{0.0};
};

GeneratedPid::GeneratedPid(PidConfig c) : impl_(new Impl), cfg_(c) {
  impl_->model->initialize();
}

GeneratedPid::~GeneratedPid() {
  impl_->model->terminate();
  delete impl_;
}

void GeneratedPid::reset() {
  impl_->model = std::make_unique<pid_ctrl>();   // 重新构造 => 状态回到初始值
  impl_->model->initialize();
  impl_->last_output = 0.0;
}

double GeneratedPid::update(double error, double measurement, double dt) {
  // 与手写版相同的入口防御：生成代码本身不检查 NaN，dt 也被固定为 0.02。
  if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(error) || !std::isfinite(measurement)) {
    return impl_->last_output;
  }
  pid_ctrl::ExtU_pid_ctrl_T u{};
  u.err = error;
  u.meas = measurement;
  u.kp = cfg_.kp;
  u.ki = cfg_.ki;
  u.kd = cfg_.kd;
  u.omin = cfg_.out_min;
  u.omax = cfg_.out_max;
  u.alpha = cfg_.d_filter_alpha;
  impl_->model->setExternalInputs(&u);
  impl_->model->step();
  impl_->last_output = impl_->model->getExternalOutputs().out;
  return impl_->last_output;
}

}  // namespace teledrive::control
