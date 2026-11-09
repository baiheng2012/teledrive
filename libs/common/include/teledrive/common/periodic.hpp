#pragma once
// 固定频率循环器，用法：
//   Periodic loop(std::chrono::milliseconds(20));   // 50Hz
//   while (running) { do_work(); loop.wait(); }
//
// 关键点：用 sleep_until(绝对时间点)，而不是 sleep_for(20ms)。
// sleep_for 会让每个周期 = 工作时间 + 20ms，误差不断累积，频率越来越低；
// sleep_until 每次对准"下一个整点"，工作耗时多少都不影响长期频率。
// 这个类用真实时间，所以不写单元测试（时间相关的逻辑在别处用 FakeClock 测）。

#include <chrono>
#include <thread>

namespace teledrive {

class Periodic {
 public:
  explicit Periodic(std::chrono::steady_clock::duration period)
      : period_(period), next_(std::chrono::steady_clock::now() + period) {}

  void wait() {
    std::this_thread::sleep_until(next_);
    next_ += period_;
    // 如果落后太多（比如调试时断点停了几秒、或系统卡顿），
    // 不要连续"疯狂补课"，直接重新对齐到现在。
    const auto now = std::chrono::steady_clock::now();
    if (now > next_ + 5 * period_) {
      next_ = now + period_;
    }
  }

 private:
  std::chrono::steady_clock::duration period_;
  std::chrono::steady_clock::time_point next_;
};

}  // namespace teledrive
