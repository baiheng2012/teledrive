#pragma once
// 把"协议帧（最多 96 字节）"映射到"多个 CAN 帧"，以及反向重组。
//
// 11 位 CAN ID（数值越小，总线仲裁越优先）：
//     bit 10..8  优先级  （0 急停 < 1 指令 < 3 心跳 < 5 遥测 < 6 其他）
//     bit  7..4  目的节点（0..14，15 = 广播）
//     bit  3..0  源节点
// 优先级放在最高位：无论节点号是多少，急停帧都能赢得仲裁。
//
// 数据区（每帧最多 8 字节）：
//     byte0 = [消息序号:4 位 | 片序号:4 位]
//     byte1 = 总片数（1..16）
//     byte2.. = 载荷（非末片必须正好 6 字节；末片 1..6 字节）

#include <array>
#include <cstddef>
#include <cstdint>

#include "teledrive/can/can_frame.hpp"

namespace teledrive::can {

inline constexpr std::uint8_t kBroadcast = 0x0F;
inline constexpr std::size_t kMaxNodes = 16;
inline constexpr std::size_t kFragHeader = 2;
inline constexpr std::size_t kFragPayload = kMaxDlc - kFragHeader;   // 6
inline constexpr std::size_t kMaxFragments = 16;
inline constexpr std::size_t kMaxCanMessage = kMaxFragments * kFragPayload;   // 96，大于最大协议帧 74

enum class Priority : std::uint8_t {
  EStop = 0,
  Command = 1,
  Heartbeat = 3,
  Telemetry = 5,
  Other = 6,
};

constexpr std::uint32_t make_id(Priority p, std::uint8_t dst, std::uint8_t src) {
  return (static_cast<std::uint32_t>(p) << 8) | (static_cast<std::uint32_t>(dst & 0x0F) << 4) |
         static_cast<std::uint32_t>(src & 0x0F);
}

struct IdFields {
  std::uint8_t priority;
  std::uint8_t dst;
  std::uint8_t src;
};

constexpr IdFields parse_id(std::uint32_t id) {
  return {static_cast<std::uint8_t>((id >> 8) & 0x07), static_cast<std::uint8_t>((id >> 4) & 0x0F),
          static_cast<std::uint8_t>(id & 0x0F)};
}

// 偷看协议帧头决定优先级。这是传输层里唯一"知道协议"的地方，
// 识别不了的数据一律最低优先级（Other）。
Priority classify(const std::uint8_t* data, std::size_t len);

struct FragmentedMessage {
  std::array<CanFrame, kMaxFragments> frames;
  std::size_t count{0};   // 0 表示输入非法（空、或超过 96 字节）
};

FragmentedMessage fragment_message(std::uint32_t id, std::uint8_t seq, const std::uint8_t* data,
                                   std::size_t len);

struct ReassemblerStats {
  std::uint64_t completed{0};
  std::uint64_t dropped_incomplete{0};   // 没收齐就被新消息顶掉
  std::uint64_t bad_frames{0};           // 格式非法
  std::uint64_t stale_frames{0};         // 重复片、已完成消息的重放
};

// 按"源节点"分别重组，每个源同一时刻只组装一条消息，新消息到来就放弃旧的（不重传）。
class CanReassembler {
 public:
  // 喂入来自 src 的一个帧。凑齐一条消息时返回 true，把内容写入 out（容量至少 kMaxCanMessage）。
  bool push(std::uint8_t src, const CanFrame& f, std::uint8_t* out, std::size_t& out_len);

  const ReassemblerStats& stats() const { return stats_; }

 private:
  struct Slot {
    bool active{false};
    bool have_done{false};
    std::uint8_t seq{0};
    std::uint8_t done_seq{0};
    std::uint8_t total{0};
    std::uint32_t mask{0};          // 已收到的片
    std::size_t last_len{0};
    std::array<std::uint8_t, kMaxCanMessage> buf{};
  };
  std::array<Slot, kMaxNodes> slots_{};
  ReassemblerStats stats_{};
};

}  // namespace teledrive::can
