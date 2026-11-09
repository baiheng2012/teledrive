#include "teledrive/node/vehicle_node.hpp"

#include <algorithm>
#include <variant>

namespace teledrive::node {

namespace {

// 每个周期最多处理多少个包。
// 如果有人往车辆端灌包，没有上限的 while 循环会让一个周期永远处理不完，
// 50Hz 的节奏就崩了；有上限则最多"处理得慢"，不会"停摆"。
constexpr int kMaxRxPerTick = 64;

// a 是否比 b "更新"？处理 16 位序列号回绕：
//   a - b 在 16 位下回绕后，当作有符号数看：正数表示 a 在 b 之后（最多领先 32767）。
//   例：a=0, b=65535 → 差为 +1 → a 更新（回绕了）
//       a=5, b=10    → 差为 -5 → a 更旧
//       a==b         → 差为 0  → 不算更新（重复包）
bool is_newer(std::uint16_t a, std::uint16_t b) {
  return static_cast<std::int16_t>(static_cast<std::uint16_t>(a - b)) > 0;
}

}  // namespace

VehicleNode::VehicleNode(transport::ITransport& transport, const IClock& clock,
                         VehicleNodeConfig config)
    : transport_(transport),
      clock_(clock),
      config_(config),
      safety_(clock, config.safety),
      model_(config.vehicle),
      heading_hold_() ,
      lidar_(config.lidar){}

void VehicleNode::tick() {
  // 1) 收包：非阻塞，取到没有为止（或达到上限）
  transport::Datagram d;
  for (int i = 0; i < kMaxRxPerTick && transport_.receive(d); ++i) {
    handle_datagram(d);
  }

  // 2) 看门狗：必须每个周期都调用，哪怕这个周期一个包都没收到。
  //    "没收到包"恰恰是看门狗要检测的情况。
  safety_.update();

  // 3) 把"经过安全层审核的指令"交给车辆模型。
  //    处理链：安全层 → 避障限速 → 航向保持 → 车辆。
  //    顺序的原则：越靠前越"安全相关"，后面的辅助只能在前面放行的范围内工作。
  auto cmd = safety_.output();
  obstacle_limited_ = false;

  if (safety_.state() == protocol::VehicleState::Driving) {
    // 避障限速：用雷达看前方，压低过快的前进指令
    if (config_.obstacle_assist_enabled) {
      const auto scan = lidar_.scan(config_.world, model_.state().pose);
      const auto r = control::limit_for_obstacles(cmd, scan, config_.limiter);
      cmd = r.command;
      obstacle_limited_ = r.limited;
    }
    // 航向保持：只改角速度
    if (config_.heading_hold_enabled) {
      const auto& s = model_.state();
      cmd.angular = static_cast<float>(
          heading_hold_.update(cmd.angular, s.pose.heading, s.linear_velocity, config_.dt));
    }
  } else {
    // 不在 Driving：让航向保持复位，避免下次起步带着旧的锁定目标
    heading_hold_.update(0.0, model_.state().pose.heading, 0.0, config_.dt);
  }
  model_.step(cmd, config_.dt);




  // 4) 按频率回传遥测（还不知道对端是谁时无处可发）
  ++tick_count_;
  const auto every = std::max<std::uint32_t>(1, config_.telemetry_every_n_ticks);
  if (peer_ && tick_count_ % every == 0) {
    send_frame(model_.telemetry(safety_.state()));
    ++stats_.tx_telemetry;
  }
}

void VehicleNode::handle_datagram(const transport::Datagram& d) {
  // 第一关：协议层校验（魔数、版本、长度、CRC、类型）
  const auto res = protocol::decode(d.data.data(), d.size);
  if (!res.ok()) {
    ++stats_.rx_bad;
    return;
  }

  // 第二关：序列号检查。
  // 如果距离上一个有效包不到 seq_resync_after，就要求 seq 必须更新；
  // 静默太久则认为对端可能重启过，接受任意 seq 并重新同步。
  const TimePoint now = clock_.now();
  const std::uint16_t seq = res.frame.seq;
  if (have_rx_ && (now - last_rx_time_) <= config_.seq_resync_after) {
    if (!is_newer(seq, last_seq_)) {
      ++stats_.rx_stale;
      return;
    }
  }
  have_rx_ = true;
  last_seq_ = seq;
  last_rx_time_ = now;
  peer_ = d.from;  // 注意：谁先发来有效包，遥测就发给谁（见文档里的"已知局限"）
  ++stats_.rx_ok;

  // 第三关：按消息类型分发
  if (const auto* cmd = std::get_if<protocol::Command>(&res.frame.payload)) {
    safety_.on_command(*cmd);  // 只有 Command 才会进入安全层、喂看门狗
  } else if (const auto* hb = std::get_if<protocol::Heartbeat>(&res.frame.payload)) {
    send_frame(*hb);           // 心跳：原样回显，操作端据此计算往返延迟；不喂看门狗
  }
  // 操作端不应发送 Telemetry，收到就忽略
}

void VehicleNode::send_frame(const protocol::Payload& payload) {
  if (!peer_) {
    return;
  }
  const auto enc = protocol::encode(tx_seq_++, payload);  // u16 自然回绕，符合设计
  transport_.send(*peer_, enc.data.data(), enc.size);
}

}  // namespace teledrive::node
