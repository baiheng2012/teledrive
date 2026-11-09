#include "teledrive/node/operator_session.hpp"

#include <chrono>
#include <variant>

namespace teledrive::node {

std::uint32_t OperatorSession::elapsed_ms() const {
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(clock_.now() - start_);
  // 转成 u32：约 49 天回绕一次。下面算 RTT 时用无符号减法，回绕时结果依然正确。
  return static_cast<std::uint32_t>(ms.count());
}

bool OperatorSession::send_payload(const protocol::Payload& payload) {
  const auto enc = protocol::encode(tx_seq_++, payload);
  return transport_.send(vehicle_, enc.data.data(), enc.size);
}

bool OperatorSession::send_command(const protocol::Command& cmd) { return send_payload(cmd); }

bool OperatorSession::send_heartbeat() {
  protocol::Heartbeat hb;
  hb.timestamp_ms = elapsed_ms();  // 带上发送时刻，车辆原样回传，我们据此算 RTT
  return send_payload(hb);
}

void OperatorSession::poll() {
  constexpr int kMaxRxPerPoll = 64;
  transport::Datagram d;
  for (int i = 0; i < kMaxRxPerPoll && transport_.receive(d); ++i) {
    const auto res = protocol::decode(d.data.data(), d.size);
    if (!res.ok()) {
      ++bad_packets_;
      continue;
    }
    if (const auto* t = std::get_if<protocol::Telemetry>(&res.frame.payload)) {
      telemetry_ = *t;
      last_telemetry_time_ = clock_.now();
    } else if (const auto* hb = std::get_if<protocol::Heartbeat>(&res.frame.payload)) {
      // 现在时刻 - 当初发送时刻 = 往返时间。两个 u32 相减，回绕也正确。
      const std::uint32_t diff = elapsed_ms() - hb->timestamp_ms;
      rtt_ = Duration(std::chrono::milliseconds(diff));
    }
  }
}

bool OperatorSession::link_alive(Duration max_age) const {
  return last_telemetry_time_.has_value() && (clock_.now() - *last_telemetry_time_) <= max_age;
}

}  // namespace teledrive::node
