#include "teledrive/video/video_packet.hpp"

#include <algorithm>
#include <cstring>

#include "teledrive/protocol/byte_io.hpp"
#include "teledrive/protocol/crc16.hpp"

namespace teledrive::video {

namespace p = teledrive::protocol;

std::vector<Packet> fragment_frame(std::uint16_t frame_id, std::uint32_t capture_ms,
                                   const std::uint8_t* data, std::size_t len) {
  std::vector<Packet> out;
  if (data == nullptr || len == 0 || len > kMaxFrameBytes) {
    return out;
  }
  // 向上取整：len=1201 -> 2 片
  const std::size_t count = (len + kMaxFragmentPayload - 1) / kMaxFragmentPayload;
  out.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t offset = i * kMaxFragmentPayload;
    const std::size_t n = std::min(kMaxFragmentPayload, len - offset);  // 最后一片可能较短

    Packet pkt(kVideoHeaderSize + n + kVideoCrcSize);
    std::size_t off = 0;
    off = p::put_u16(pkt.data(), off, kVideoMagic);
    off = p::put_u8(pkt.data(), off, kVideoVersion);
    off = p::put_u8(pkt.data(), off, static_cast<std::uint8_t>(count));
    off = p::put_u8(pkt.data(), off, static_cast<std::uint8_t>(i));
    off = p::put_u16(pkt.data(), off, frame_id);
    off = p::put_u32(pkt.data(), off, capture_ms);
    off = p::put_u16(pkt.data(), off, static_cast<std::uint16_t>(n));
    std::memcpy(pkt.data() + off, data + offset, n);
    off += n;
    p::put_u16(pkt.data(), off, p::crc16(pkt.data(), off));   // CRC 覆盖头部和载荷
    out.push_back(std::move(pkt));
  }
  return out;
}

void FrameReassembler::start_frame(std::uint16_t frame_id, std::uint8_t count,
                                   std::uint32_t capture_ms) {
  have_current_ = true;
  cur_id_ = frame_id;
  cur_count_ = count;
  cur_capture_ms_ = capture_ms;
  parts_.assign(count, {});
  have_.assign(count, false);
  received_ = 0;
}

bool FrameReassembler::push(const std::uint8_t* d, std::size_t len, VideoFrame& out) {
  // ---- 1) 解析与校验：网络数据不可信，先查长度再读字段 ----
  if (d == nullptr || len < kVideoHeaderSize + kVideoCrcSize) {
    ++stats_.bad_packets;
    return false;
  }
  const std::uint16_t magic = p::get_u16(d, 0);
  const std::uint8_t version = p::get_u8(d, 2);
  const std::uint8_t count = p::get_u8(d, 3);
  const std::uint8_t index = p::get_u8(d, 4);
  const std::uint16_t frame_id = p::get_u16(d, 5);
  const std::uint32_t capture_ms = p::get_u32(d, 7);
  const std::uint16_t plen = p::get_u16(d, 11);

  if (magic != kVideoMagic || version != kVideoVersion || count == 0 || index >= count ||
      plen == 0 || plen > kMaxFragmentPayload || len != kVideoHeaderSize + plen + kVideoCrcSize) {
    ++stats_.bad_packets;
    return false;
  }
  const std::size_t body = kVideoHeaderSize + plen;
  if (p::crc16(d, body) != p::get_u16(d, body)) {
    ++stats_.bad_packets;
    return false;
  }

  // ---- 2) 判断这个分片属于哪一帧 ----
  if (have_current_) {
    if (frame_id == cur_id_) {
      // 同一帧的分片，总片数和时间戳必须一致，否则数据自相矛盾
      if (count != cur_count_ || capture_ms != cur_capture_ms_) {
        ++stats_.bad_packets;
        return false;
      }
    } else if (frame_id_newer(frame_id, cur_id_)) {
      ++stats_.dropped_incomplete;                 // 更新的帧来了：放弃没收齐的旧帧
      start_frame(frame_id, count, capture_ms);
    } else {
      ++stats_.stale_packets;                      // 旧帧的迟到分片
      return false;
    }
  } else {
    if (have_last_ && !frame_id_newer(frame_id, last_id_)) {
      ++stats_.stale_packets;                      // 已完成帧的重复分片，或更旧的帧
      return false;
    }
    start_frame(frame_id, count, capture_ms);
  }

  // ---- 3) 存入分片 ----
  if (have_[index]) {
    ++stats_.stale_packets;                        // 重复分片
    return false;
  }
  parts_[index].assign(d + kVideoHeaderSize, d + body);
  have_[index] = true;
  ++received_;
  if (received_ < cur_count_) {
    return false;
  }

  // ---- 4) 收齐了：按序拼接 ----
  out.frame_id = cur_id_;
  out.capture_ms = cur_capture_ms_;
  out.data.clear();
  for (const auto& part : parts_) {
    out.data.insert(out.data.end(), part.begin(), part.end());
  }
  have_current_ = false;
  have_last_ = true;
  last_id_ = cur_id_;
  parts_.clear();
  ++stats_.completed;
  return true;
}

}  // namespace teledrive::video
