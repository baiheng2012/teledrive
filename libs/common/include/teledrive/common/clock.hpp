#pragma once
#include <chrono>

namespace teledrive {

using TimePoint = std::chrono::steady_clock::time_point;
using Duration  = std::chrono::steady_clock::duration;

// 时钟接口：生产代码用真实时钟，测试用假时钟
class IClock {
 public:
  virtual ~IClock() = default;
  virtual TimePoint now() const = 0;
};

class SteadyClock final : public IClock {
 public:
  TimePoint now() const override { return std::chrono::steady_clock::now(); }
};

// 测试专用：时间只在你手动推进时才前进
class FakeClock final : public IClock {
 public:
  TimePoint now() const override { return now_; }
  void advance(Duration d) { now_ += d; }

 private:
  TimePoint now_{};
};

}  // namespace teledrive
