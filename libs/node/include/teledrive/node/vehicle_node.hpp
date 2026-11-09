#pragma once
// 车辆端节点：把 传输 → 协议 → 安全 → 车辆模型 → 遥测 串成一个控制周期。
//
// 使用方式：外部以固定频率（50Hz）反复调用 tick()。
// 本类不读真实时间、不 sleep，也不创建 socket：
//   - 时间来自注入的 IClock（测试用 FakeClock）
//   - 网络来自注入的 ITransport（测试用 MockTransport）
// 所以整个车辆端逻辑可以在单元测试里确定地、瞬间地运行。

#include <cstdint>
#include <optional>

#include "teledrive/common/clock.hpp"
#include "teledrive/protocol/messages.hpp"
#include "teledrive/safety/safety_controller.hpp"
#include "teledrive/sim/diff_drive.hpp"
#include "teledrive/transport/transport.hpp"

#include "teledrive/control/heading_hold.hpp"




namespace teledrive::node {

struct VehicleNodeConfig {
  safety::SafetyConfig safety{};
  sim::VehicleParams vehicle{};
  double dt{0.02};                              // 控制周期（秒），要与调用 tick() 的频率一致
  std::uint32_t telemetry_every_n_ticks{5};     // 50Hz / 5 = 10Hz 遥测
  Duration seq_resync_after{std::chrono::seconds(1)};  // 静默超过此时间，接受任意 seq

  bool heading_hold_enabled{true};   // 是否启用航向保持辅助

};

// 运行统计：用于终端显示，也方便测试断言"坏包确实被拒绝了"
struct VehicleNodeStats {
  std::uint64_t rx_ok{0};         // 通过校验并被接受的包
  std::uint64_t rx_bad{0};        // 协议层解码失败（CRC 错误、格式错误等）
  std::uint64_t rx_stale{0};      // 解码正确但 seq 过旧或重复
  std::uint64_t tx_telemetry{0};  // 已发送的遥测包
};

class VehicleNode {
 public:
  // 持有 transport 和 clock 的引用（不拥有），要求它们比本对象活得久。
  VehicleNode(transport::ITransport& transport, const IClock& clock,
              VehicleNodeConfig config = {});

  // 一个控制周期：收包 → 看门狗检查 → 推进车辆 → （按频率）发遥测
  void tick();

  const safety::SafetyController& safety() const { return safety_; }
  const sim::DiffDriveModel& model() const { return model_; }
  const VehicleNodeStats& stats() const { return stats_; }

 private:
  void handle_datagram(const transport::Datagram& d);
  void send_frame(const protocol::Payload& payload);

  transport::ITransport& transport_;
  const IClock& clock_;
  VehicleNodeConfig config_;
  safety::SafetyController safety_;
  sim::DiffDriveModel model_;
  control::HeadingHold heading_hold_;
  VehicleNodeStats stats_{};

  // 对端地址：以"最近一个有效包的来源"为准，遥测发给它
  std::optional<transport::Endpoint> peer_;

  // 序列号跟踪（接收方向）
  bool have_rx_{false};
  std::uint16_t last_seq_{0};
  TimePoint last_rx_time_{};

  std::uint16_t tx_seq_{0};       // 发送方向的序列号
  std::uint32_t tick_count_{0};
};

}  // namespace teledrive::node
