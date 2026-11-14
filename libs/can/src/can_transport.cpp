#include "teledrive/can/can_transport.hpp"

namespace teledrive::can {

bool CanTransport::send(const transport::Endpoint& to, const std::uint8_t* data, std::size_t len) {
  if (!valid_ || data == nullptr || len == 0 || len > kMaxCanMessage || to.port > kBroadcast) {
    return false;
  }
  const std::uint32_t id = make_id(classify(data, len), static_cast<std::uint8_t>(to.port), node_);
  const FragmentedMessage msg = fragment_message(id, tx_seq_, data, len);
  tx_seq_ = static_cast<std::uint8_t>((tx_seq_ + 1) & 0x0F);

  for (std::size_t i = 0; i < msg.count; ++i) {
    // 一片发不出去就整条放弃：接收端会在下一条消息到来时丢掉残缺的这条
    if (!bus_.send_frame(msg.frames[i])) return false;
  }
  return true;
}

bool CanTransport::receive(transport::Datagram& out) {
  if (!valid_) return false;
  CanFrame f;
  // 每次调用最多处理 64 帧，防止总线被灌满时拖住控制循环
  for (int i = 0; i < 64 && bus_.receive_frame(f); ++i) {
    const IdFields id = parse_id(f.id);
    if (id.dst != node_ && id.dst != kBroadcast) continue;   // 不是发给我的
    if (id.src == node_) continue;                            // 自己的回环

    std::size_t len = 0;
    if (reassembler_.push(id.src, f, out.data.data(), len)) {
      out.size = len;
      out.from = can_endpoint(id.src);
      return true;   // 一次返回一条消息，剩下的帧留给下次调用
    }
  }
  return false;
}

}  // namespace teledrive::can
