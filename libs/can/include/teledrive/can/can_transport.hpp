#pragma once
// 基于 CAN 的 ITransport 实现：上层（VehicleNode / OperatorSession）完全不用改。
// 地址约定：Endpoint{ip = 0, port = 节点号}；节点号 0..14，15 为广播。
// 收到的 Datagram.from 同样是 Endpoint{0, 源节点号}。

#include <cstdint>

#include "teledrive/can/can_frame.hpp"
#include "teledrive/can/can_mapping.hpp"
#include "teledrive/transport/transport.hpp"

namespace teledrive::can {

inline transport::Endpoint can_endpoint(std::uint8_t node) { return transport::Endpoint{0, node}; }

class CanTransport final : public transport::ITransport {
 public:
  // node_id 必须是 0..14（15 留给广播），否则本对象无效，send/receive 一律失败
  CanTransport(ICanBus& bus, std::uint8_t node_id)
      : bus_(bus), node_(node_id), valid_(node_id < kBroadcast) {}

  // to.port 是目的节点号；数据长度 1..96
  bool send(const transport::Endpoint& to, const std::uint8_t* data, std::size_t len) override;
  bool receive(transport::Datagram& out) override;

  const ReassemblerStats& stats() const { return reassembler_.stats(); }

 private:
  ICanBus& bus_;
  std::uint8_t node_;
  bool valid_;
  std::uint8_t tx_seq_{0};   // 4 位消息序号，自然回绕
  CanReassembler reassembler_;
};

}  // namespace teledrive::can
