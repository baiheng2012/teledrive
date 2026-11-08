#pragma once
// 看门狗：监视"某件事是否持续发生"。
// 用法：被监视的事件每次发生就调用 feed()（喂狗）；
//       控制循环里调用 expired() 检查，超过 timeout 没喂就返回 true。
// 时间来自注入的 IClock：生产环境用真实时钟，测试用 FakeClock，
// 因此测试"200ms 超时"不需要真的等 200ms。

#include "teledrive/common/clock.hpp"

namespace teledrive::safety {

class Watchdog {
 public:
  // 持有的是 clock 的引用，不拥有它：clock 必须比 Watchdog 活得久。
  // 由于有引用成员，这个类不可赋值，这是有意的（看门狗不该被"换时钟"）。
  Watchdog(const IClock& clock, Duration timeout) : clock_(clock), timeout_(timeout) {}

  // 喂狗：记录当前时间，并让看门狗开始（或继续）工作
  void feed() {
    last_feed_ = clock_.now();
    running_ = true;
  }

  // 停止监视：之后 expired() 恒为 false，直到再次 feed()
  void stop() { running_ = false; }

  bool running() const { return running_; }

  // 是否超时。用 ">" 而不是 ">="：恰好等于 timeout 时不算超时，
  // 边界语义明确，测试里会钉死这一点。
  bool expired() const { return running_ && (clock_.now() - last_feed_) > timeout_; }

 private:
  const IClock& clock_;
  Duration timeout_;
  TimePoint last_feed_{};
  bool running_{false};
};

}  // namespace teledrive::safety
