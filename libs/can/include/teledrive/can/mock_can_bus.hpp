#pragma once
// 内存里的 CAN 总线，只用于测试。
// 真实 CAN 是广播总线：join() 之后，任何一个节点发的帧，其余所有节点都收得到（自己收不到）。
// 可以注入丢帧和任意帧，sent_log() 记录自己发过的所有帧，用来检查 ID 和优先级。

#include <deque>
#include <vector>

#include "teledrive/can/can_frame.hpp"

namespace teledrive::can {

class MockCanBus final : public ICanBus {
 public:
  // 把两个节点接到同一条总线上（互相可见）。三个节点就 join 三对。
  static void join(MockCanBus& a, MockCanBus& b) {
    a.peers_.push_back(&b);
    b.peers_.push_back(&a);
  }

  void drop_next_frames(int n) { drop_remaining_ = n; }   // 接下来 n 帧"发出去了"但被网络吞掉
  void inject(const CanFrame& f) { inbox_.push_back(f); }
  const std::vector<CanFrame>& sent_log() const { return sent_; }

  bool send_frame(const CanFrame& f) override;
  bool receive_frame(CanFrame& out) override;

 private:
  std::vector<MockCanBus*> peers_;
  std::deque<CanFrame> inbox_;
  std::vector<CanFrame> sent_;
  int drop_remaining_{0};
};

}  // namespace teledrive::can
