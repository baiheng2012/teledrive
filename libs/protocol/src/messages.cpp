#include "teledrive/protocol/messages.hpp"

#include "teledrive/protocol/byte_io.hpp"
#include "teledrive/protocol/crc16.hpp"

namespace teledrive::protocol {

namespace {

// 各类型载荷的固定大小
constexpr std::size_t kHeartbeatSize = 4;
constexpr std::size_t kCommandSize   = 9;
constexpr std::size_t kTelemetrySize = 18;

// ---- 载荷编码：返回写入后的 offset ----
std::size_t write_payload(std::uint8_t* buf, std::size_t off, const Heartbeat& m) {
  return put_u32(buf, off, m.timestamp_ms);
}
std::size_t write_payload(std::uint8_t* buf, std::size_t off, const Command& m) {
  off = put_f32(buf, off, m.linear);
  off = put_f32(buf, off, m.angular);
  return put_u8(buf, off, m.flags);
}
std::size_t write_payload(std::uint8_t* buf, std::size_t off, const Telemetry& m) {
  off = put_f32(buf, off, m.x);
  off = put_f32(buf, off, m.y);
  off = put_f32(buf, off, m.heading);
  off = put_f32(buf, off, m.speed);
  off = put_u8(buf, off, static_cast<std::uint8_t>(m.state));
  return put_u8(buf, off, m.battery_pct);
}

// 通过类型返回对应的 MsgType 枚举值（重载 + 编译期分派）
constexpr MsgType type_of(const Heartbeat&) { return MsgType::Heartbeat; }
constexpr MsgType type_of(const Command&)   { return MsgType::Command; }
constexpr MsgType type_of(const Telemetry&) { return MsgType::Telemetry; }

}  // namespace

EncodedFrame encode(std::uint16_t seq, const Payload& payload) {
  EncodedFrame out;
  std::uint8_t* buf = out.data.data();

  // 先把载荷写到头部之后的位置（偏移 kHeaderSize），同时得到载荷类型和长度。
  // std::visit：根据 variant 里当前存的实际类型，调用对应的重载。
  MsgType type{};
  std::size_t end = kHeaderSize;
  std::visit(
      [&](const auto& msg) {
        type = type_of(msg);
        end = write_payload(buf, kHeaderSize, msg);
      },
      payload);
  const std::size_t payload_len = end - kHeaderSize;

  // 再回头写头部
  std::size_t off = 0;
  off = put_u16(buf, off, kMagic);
  off = put_u8(buf, off, kVersion);
  off = put_u8(buf, off, static_cast<std::uint8_t>(type));
  off = put_u16(buf, off, seq);
  off = put_u16(buf, off, static_cast<std::uint16_t>(payload_len));

  // 最后对"头部 + 载荷"算 CRC，附在末尾
  const std::uint16_t crc = crc16(buf, end);
  end = put_u16(buf, end, crc);

  out.size = end;
  return out;
}

DecodeResult decode(const std::uint8_t* data, std::size_t len) {
  DecodeResult r;
  auto fail = [&r](DecodeError e) {
    r.error = e;
    return r;
  };

  // 检查顺序很重要：先查长度，再读字段，防止越界读取
  if (data == nullptr || len < kHeaderSize + kCrcSize) return fail(DecodeError::TooShort);

  if (get_u16(data, 0) != kMagic) return fail(DecodeError::BadMagic);
  if (get_u8(data, 2) != kVersion) return fail(DecodeError::BadVersion);

  const std::uint8_t type_raw = get_u8(data, 3);
  const std::uint16_t seq = get_u16(data, 4);
  const std::uint16_t payload_len = get_u16(data, 6);

  // length 字段来自网络，不可信：必须限制上限，并且和实际收到的字节数严格吻合
  if (payload_len > kMaxPayload) return fail(DecodeError::BadLength);
  if (len != kHeaderSize + payload_len + kCrcSize) return fail(DecodeError::BadLength);

  // 校验 CRC
  const std::size_t body_len = kHeaderSize + payload_len;
  const std::uint16_t expected = get_u16(data, body_len);
  if (crc16(data, body_len) != expected) return fail(DecodeError::BadCrc);

  // 按类型解析载荷
  const std::uint8_t* p = data + kHeaderSize;
  r.frame.seq = seq;
  switch (static_cast<MsgType>(type_raw)) {
    case MsgType::Heartbeat: {
      if (payload_len != kHeartbeatSize) return fail(DecodeError::BadPayloadSize);
      Heartbeat m;
      m.timestamp_ms = get_u32(p, 0);
      r.frame.payload = m;
      break;
    }
    case MsgType::Command: {
      if (payload_len != kCommandSize) return fail(DecodeError::BadPayloadSize);
      Command m;
      m.linear = get_f32(p, 0);
      m.angular = get_f32(p, 4);
      m.flags = get_u8(p, 8);
      r.frame.payload = m;
      break;
    }
    case MsgType::Telemetry: {
      if (payload_len != kTelemetrySize) return fail(DecodeError::BadPayloadSize);
      Telemetry m;
      m.x = get_f32(p, 0);
      m.y = get_f32(p, 4);
      m.heading = get_f32(p, 8);
      m.speed = get_f32(p, 12);
      m.state = static_cast<VehicleState>(get_u8(p, 16));
      m.battery_pct = get_u8(p, 17);
      r.frame.payload = m;
      break;
    }
    default:
      return fail(DecodeError::UnknownType);
  }
  return r;
}

}  // namespace teledrive::protocol
