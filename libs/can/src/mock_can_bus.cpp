#include "teledrive/can/mock_can_bus.hpp"

namespace teledrive::can {

bool MockCanBus::send_frame(const CanFrame& f) {
  // 和真实驱动一样的参数检查
  if (f.dlc > kMaxDlc || f.id > kMaxStandardId) return false;
  sent_.push_back(f);
  if (drop_remaining_ > 0) {
    --drop_remaining_;
    return true;   // 发送方不知道丢了
  }
  for (MockCanBus* p : peers_) p->inbox_.push_back(f);
  return true;
}

bool MockCanBus::receive_frame(CanFrame& out) {
  if (inbox_.empty()) return false;
  out = inbox_.front();
  inbox_.pop_front();
  return true;
}

}  // namespace teledrive::can
