#pragma once
// 操作端的通信会话：负责"发指令、发心跳、收遥测、算延迟"。
// 和 VehicleNode 一样，不依赖真实网络或真实时间，可以用 Mock 测试。
// 键盘、终端显示这些"脏活"不在这里，在 apps/operator_cli 里。

#include <cstdint>
#include <optional>

#include "teledrive/common/clock.hpp"
#include "teledrive/protocol/messages.hpp"
#include "teledrive/transport/transport.hpp"

namespace teledrive::node {

class OperatorSession {
 public:
  OperatorSession(transport::ITransport& transport, const IClock& clock,
                  transport::Endpoint vehicle)
      : transport_(transport), clock_(clock), vehicle_(vehicle), start_(clock.now()) {}

  // 发送一条指令 / 一个心跳。两者共用同一个递增的 seq（车辆端按"来源整体"检查新旧）。
  bool send_command(const protocol::Command& cmd);
  bool send_heartbeat();

  // 每个周期调用：取出所有已到达的数据，更新遥测和 RTT
  void poll();

  // 最近一次收到的遥测；从未收到则为空
  std::optional<protocol::Telemetry> latest_telemetry() const { return telemetry_; }

  // 最近一次测得的往返延迟（Round-Trip Time）
  std::optional<Duration> rtt() const { return rtt_; }

  // 最近 max_age 内是否收到过遥测（用来显示"连接正常/断开"）
  bool link_alive(Duration max_age) const;

  std::uint64_t bad_packets() const { return bad_packets_; }

 private:
  bool send_payload(const protocol::Payload& payload);
  std::uint32_t elapsed_ms() const;

  transport::ITransport& transport_;
  const IClock& clock_;
  transport::Endpoint vehicle_;
  TimePoint start_;                      // 会话开始时间，心跳时间戳 = 相对此刻的毫秒数

  std::uint16_t tx_seq_{0};
  std::optional<protocol::Telemetry> telemetry_;
  std::optional<TimePoint> last_telemetry_time_;
  std::optional<Duration> rtt_;
  std::uint64_t bad_packets_{0};
};

}  // namespace teledrive::node
