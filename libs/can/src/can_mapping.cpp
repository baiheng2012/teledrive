#include "teledrive/can/can_mapping.hpp"

#include <algorithm>
#include <cstring>

#include "teledrive/protocol/byte_io.hpp"
#include "teledrive/protocol/messages.hpp"

namespace teledrive::can {

namespace p = teledrive::protocol;

Priority classify(const std::uint8_t* d, std::size_t len) {
  if (d == nullptr || len < p::kHeaderSize || p::get_u16(d, 0) != p::kMagic) {
    return Priority::Other;
  }
  switch (static_cast<p::MsgType>(d[3])) {
    case p::MsgType::Command:
      // Command 载荷 9 字节，flags 是最后一个载荷字节。带急停标志的指令升到最高优先级
      if (len == p::kHeaderSize + 9 + p::kCrcSize && (d[p::kHeaderSize + 8] & p::kFlagEStop)) {
        return Priority::EStop;
      }
      return Priority::Command;
    case p::MsgType::Heartbeat: return Priority::Heartbeat;
    case p::MsgType::Telemetry: return Priority::Telemetry;
  }
  return Priority::Other;   // 未知类型
}

FragmentedMessage fragment_message(std::uint32_t id, std::uint8_t seq, const std::uint8_t* data,
                                   std::size_t len) {
  FragmentedMessage m;
  if (data == nullptr || len == 0 || len > kMaxCanMessage) return m;

  const std::size_t count = (len + kFragPayload - 1) / kFragPayload;   // 向上取整
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t n = std::min(kFragPayload, len - i * kFragPayload);   // 末片可能较短
    CanFrame& f = m.frames[i];
    f.id = id;
    f.dlc = static_cast<std::uint8_t>(kFragHeader + n);
    f.data[0] = static_cast<std::uint8_t>(((seq & 0x0F) << 4) | i);
    f.data[1] = static_cast<std::uint8_t>(count);
    std::memcpy(f.data.data() + kFragHeader, data + i * kFragPayload, n);
  }
  m.count = count;
  return m;
}

bool CanReassembler::push(std::uint8_t src, const CanFrame& f, std::uint8_t* out,
                          std::size_t& out_len) {
  // ---- 1) 校验：总线上的数据同样不可信 ----
  if (src >= kMaxNodes || f.dlc < kFragHeader || f.dlc > kMaxDlc) {
    ++stats_.bad_frames;
    return false;
  }
  const std::uint8_t seq = f.data[0] >> 4;
  const std::uint8_t idx = f.data[0] & 0x0F;
  const std::uint8_t total = f.data[1];
  const std::size_t n = f.dlc - kFragHeader;
  const bool last = (idx + 1 == total);
  // 非末片必须正好 6 字节，这样总长度才能由末片长度推出；末片至少 1 字节
  if (total == 0 || total > kMaxFragments || idx >= total || (last ? n == 0 : n != kFragPayload)) {
    ++stats_.bad_frames;
    return false;
  }

  // ---- 2) 判断属于哪条消息 ----
  Slot& s = slots_[src];
  if (s.active && s.seq != seq) {   // 新消息开始：放弃没收齐的旧消息
    ++stats_.dropped_incomplete;
    s.active = false;
  }
  if (!s.active) {
    if (s.have_done && s.done_seq == seq) {   // 刚完成的那条消息的重放
      ++stats_.stale_frames;
      return false;
    }
    s.active = true;
    s.seq = seq;
    s.total = total;
    s.mask = 0;
    s.last_len = 0;
  } else if (s.total != total) {   // 同一条消息的各片声称不同的总片数：自相矛盾
    ++stats_.bad_frames;
    return false;
  }

  // ---- 3) 存片 ----
  const std::uint32_t bit = 1u << idx;
  if (s.mask & bit) {
    ++stats_.stale_frames;   // 重复片
    return false;
  }
  std::memcpy(s.buf.data() + idx * kFragPayload, f.data.data() + kFragHeader, n);
  s.mask |= bit;
  if (last) s.last_len = n;
  if (s.mask != ((1u << total) - 1)) return false;   // 还没收齐

  // ---- 4) 收齐 ----
  out_len = (total - 1) * kFragPayload + s.last_len;
  std::memcpy(out, s.buf.data(), out_len);
  s.active = false;
  s.have_done = true;
  s.done_seq = seq;
  ++stats_.completed;
  return true;
}

}  // namespace teledrive::can
