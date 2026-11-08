#include "teledrive/transport/mock_transport.hpp"

#include <cstring>

namespace teledrive::transport {

void MockTransport::inject(const Endpoint& from, const std::uint8_t* data, std::size_t len) {
  if (len > kMaxDatagramSize) {
    return;
  }
  Datagram d;
  if (len > 0 && data != nullptr) {
    std::memcpy(d.data.data(), data, len);
  }
  d.size = len;
  d.from = from;
  inbox_.push_back(d);
}

bool MockTransport::send(const Endpoint& /*to*/, const std::uint8_t* data, std::size_t len) {
  // 和 UdpTransport 保持一致的参数检查，这样 mock 和真实实现行为相同
  if (data == nullptr || len == 0 || len > kMaxDatagramSize) {
    return false;
  }
  ++sent_count_;

  // 模拟丢包：对发送方来说"发送成功"，但数据凭空消失
  if (drop_remaining_ > 0) {
    --drop_remaining_;
    return true;
  }
  if (peer_ == nullptr) {
    return false;  // 没有连接对端
  }
  // 数据到达对端，来源地址是自己
  peer_->inject(self_, data, len);
  return true;
}

bool MockTransport::receive(Datagram& out) {
  if (inbox_.empty()) {
    return false;  // 和非阻塞 socket 一样：没数据就立刻返回
  }
  out = inbox_.front();
  inbox_.pop_front();
  return true;
}

}  // namespace teledrive::transport
