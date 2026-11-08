#include <gtest/gtest.h>
#include <teledrive/protocol/messages.hpp>
#include <teledrive/protocol/crc16.hpp>
#include <vector>

using namespace teledrive::protocol;

namespace {
// 辅助函数：把 EncodedFrame 复制成 vector，方便在测试里篡改字节
std::vector<std::uint8_t> bytes_of(const EncodedFrame& f) {
  return {f.data.begin(), f.data.begin() + f.size};
}
}  // namespace

// ---------- 正常往返：encode 再 decode，得到原值 ----------

TEST(Messages, HeartbeatRoundTrip) {
  auto enc = encode(7, Heartbeat{123456});
  auto res = decode(enc.data.data(), enc.size);
  ASSERT_TRUE(res.ok());
  EXPECT_EQ(res.frame.seq, 7);
  EXPECT_EQ(std::get<Heartbeat>(res.frame.payload).timestamp_ms, 123456u);
}

TEST(Messages, CommandRoundTrip) {
  Command c;
  c.linear = 1.5f;
  c.angular = -0.75f;
  c.flags = kFlagArm | kFlagEStop;
  auto enc = encode(65535, c);  // seq 取 u16 最大值，顺便测边界
  auto res = decode(enc.data.data(), enc.size);
  ASSERT_TRUE(res.ok());
  EXPECT_EQ(res.frame.seq, 65535);
  const auto& got = std::get<Command>(res.frame.payload);
  EXPECT_FLOAT_EQ(got.linear, 1.5f);
  EXPECT_FLOAT_EQ(got.angular, -0.75f);
  EXPECT_EQ(got.flags, kFlagArm | kFlagEStop);
}

TEST(Messages, TelemetryRoundTrip) {
  Telemetry t;
  t.x = 10.25f; t.y = -3.5f; t.heading = 1.57f; t.speed = 0.8f;
  t.state = VehicleState::Driving;
  t.battery_pct = 87;
  auto enc = encode(1, t);
  auto res = decode(enc.data.data(), enc.size);
  ASSERT_TRUE(res.ok());
  const auto& got = std::get<Telemetry>(res.frame.payload);
  EXPECT_FLOAT_EQ(got.x, 10.25f);
  EXPECT_FLOAT_EQ(got.y, -3.5f);
  EXPECT_FLOAT_EQ(got.heading, 1.57f);
  EXPECT_FLOAT_EQ(got.speed, 0.8f);
  EXPECT_EQ(got.state, VehicleState::Driving);
  EXPECT_EQ(got.battery_pct, 87);
}

// ---------- 帧大小：编码后的长度必须符合协议文档 ----------

TEST(Messages, EncodedSizesMatchSpec) {
  EXPECT_EQ(encode(0, Heartbeat{}).size, 8u + 4u + 2u);
  EXPECT_EQ(encode(0, Command{}).size, 8u + 9u + 2u);
  EXPECT_EQ(encode(0, Telemetry{}).size, 8u + 18u + 2u);
}

// ---------- 字节序：钉死线上格式 ----------
// 这个测试把"小端"写成了铁律：以后谁改了字节序，测试立刻报警。

TEST(Messages, WireFormatIsLittleEndian) {
  auto enc = encode(0x0102, Heartbeat{0x0A0B0C0D});
  const auto* d = enc.data.data();
  EXPECT_EQ(d[0], 0x44);  // magic 0x5444 的低字节在前
  EXPECT_EQ(d[1], 0x54);
  EXPECT_EQ(d[2], kVersion);
  EXPECT_EQ(d[3], static_cast<std::uint8_t>(MsgType::Heartbeat));
  EXPECT_EQ(d[4], 0x02);  // seq 0x0102：低字节 0x02 在前
  EXPECT_EQ(d[5], 0x01);
  EXPECT_EQ(d[6], 0x04);  // length = 4
  EXPECT_EQ(d[7], 0x00);
  EXPECT_EQ(d[8], 0x0D);  // timestamp 0x0A0B0C0D：低字节在前
  EXPECT_EQ(d[9], 0x0C);
  EXPECT_EQ(d[10], 0x0B);
  EXPECT_EQ(d[11], 0x0A);
}

// ---------- 坏包：每种错误码至少一个用例 ----------

TEST(Messages, RejectsNullOrTooShort) {
  EXPECT_EQ(decode(nullptr, 0).error, DecodeError::TooShort);
  std::uint8_t tiny[5] = {};
  EXPECT_EQ(decode(tiny, 5).error, DecodeError::TooShort);
}

TEST(Messages, RejectsBadMagic) {
  auto v = bytes_of(encode(1, Heartbeat{1}));
  v[0] ^= 0xFF;
  EXPECT_EQ(decode(v.data(), v.size()).error, DecodeError::BadMagic);
}

TEST(Messages, RejectsBadVersion) {
  auto v = bytes_of(encode(1, Heartbeat{1}));
  v[2] = 99;
  EXPECT_EQ(decode(v.data(), v.size()).error, DecodeError::BadVersion);
}

TEST(Messages, RejectsCorruptedPayloadByCrc) {
  auto v = bytes_of(encode(1, Command{}));
  v[10] ^= 0x01;  // 翻转载荷里的一个比特
  EXPECT_EQ(decode(v.data(), v.size()).error, DecodeError::BadCrc);
}

TEST(Messages, RejectsCorruptedCrcItself) {
  auto v = bytes_of(encode(1, Command{}));
  v.back() ^= 0x01;
  EXPECT_EQ(decode(v.data(), v.size()).error, DecodeError::BadCrc);
}

TEST(Messages, RejectsTruncatedFrame) {
  auto v = bytes_of(encode(1, Command{}));
  // 少收 1 个字节：长度字段与实际长度不一致
  EXPECT_EQ(decode(v.data(), v.size() - 1).error, DecodeError::BadLength);
}

TEST(Messages, RejectsTrailingGarbage) {
  auto v = bytes_of(encode(1, Command{}));
  v.push_back(0x00);  // 多出一个字节
  EXPECT_EQ(decode(v.data(), v.size()).error, DecodeError::BadLength);
}

TEST(Messages, RejectsOversizedLengthField) {
  auto v = bytes_of(encode(1, Heartbeat{1}));
  v[6] = 0xFF; v[7] = 0xFF;  // length = 65535，远超 kMaxPayload
  EXPECT_EQ(decode(v.data(), v.size()).error, DecodeError::BadLength);
}

// 构造"CRC 正确但类型未知"的包：必须重新算 CRC，否则会先被 BadCrc 拦下
TEST(Messages, RejectsUnknownType) {
  auto v = bytes_of(encode(1, Heartbeat{1}));
  v[3] = 0x7F;                                  // 未知类型
  const auto crc = crc16(v.data(), v.size() - 2);
  v[v.size() - 2] = static_cast<std::uint8_t>(crc & 0xFF);
  v[v.size() - 1] = static_cast<std::uint8_t>(crc >> 8);
  EXPECT_EQ(decode(v.data(), v.size()).error, DecodeError::UnknownType);
}
