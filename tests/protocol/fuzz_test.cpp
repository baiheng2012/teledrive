// 协议层的"性质测试 / 模糊测试"。
// 随机数种子固定 => 每次运行完全相同，失败时能复现。
// 建议在 ASan/UBSan 构建下运行：随机输入最容易触发越界读写。
#include <gtest/gtest.h>
#include <teledrive/protocol/byte_io.hpp>
#include <teledrive/protocol/crc16.hpp>
#include <teledrive/protocol/messages.hpp>

#include <cstring>
#include <random>
#include <vector>

using namespace teledrive::protocol;

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes bytes_of(const EncodedFrame& f) { return {f.data.begin(), f.data.begin() + f.size}; }

// float 与位模式互转。比较"位模式"而不是比较 float 值：NaN != NaN，直接比较会误报
std::uint32_t bits_of(float f) {
  std::uint32_t b;
  std::memcpy(&b, &f, sizeof b);
  return b;
}
float float_of(std::uint32_t b) {
  float f;
  std::memcpy(&f, &b, sizeof f);
  return f;
}

// 构造一个"魔数、版本、长度、CRC 全部合法，但类型和载荷随机"的帧。
// 纯随机字节几乎都会在魔数检查处被拒绝，走不到解码器深处；这个生成器能覆盖更深的路径。
Bytes random_wellformed_frame(std::mt19937& rng) {
  static const std::size_t kValidSizes[] = {4, 9, 18};   // 三种消息的真实载荷长度
  const auto type = static_cast<std::uint8_t>(1 + rng() % 3);
  // 70% 用真实长度（有机会被接受），30% 用随机长度（应当报 BadPayloadSize）
  const std::size_t len =
      (rng() % 10 < 7) ? kValidSizes[rng() % 3] : static_cast<std::size_t>(rng() % (kMaxPayload + 1));

  Bytes f(kHeaderSize + len + kCrcSize);
  std::size_t off = 0;
  off = put_u16(f.data(), off, kMagic);
  off = put_u8(f.data(), off, kVersion);
  off = put_u8(f.data(), off, type);
  off = put_u16(f.data(), off, static_cast<std::uint16_t>(rng()));
  off = put_u16(f.data(), off, static_cast<std::uint16_t>(len));
  for (std::size_t i = 0; i < len; ++i) f[off + i] = static_cast<std::uint8_t>(rng());
  off += len;
  put_u16(f.data(), off, crc16(f.data(), off));   // 最后算 CRC，覆盖头部和载荷
  return f;
}

}  // namespace

// 性质 1：随机字节不崩溃，也不会被误接受
TEST(ProtocolFuzz, RandomBytesAreNeverAccepted) {
  std::mt19937 rng(12345);
  Bytes buf(kMaxFrameSize + 32);
  int accepted = 0;
  for (int i = 0; i < 100000; ++i) {
    const std::size_t len = rng() % buf.size();   // 长度 0 也包含在内
    for (std::size_t j = 0; j < len; ++j) buf[j] = static_cast<std::uint8_t>(rng());
    if (decode(buf.data(), len).ok()) ++accepted;
  }
  EXPECT_EQ(accepted, 0);
}

// 性质 2：结构合法的随机帧。被接受 => 重新编码得到原始字节；被拒绝 => 原因只能是载荷长度不符
TEST(ProtocolFuzz, WellformedRandomFramesAreAcceptedOrRejectedCleanly) {
  std::mt19937 rng(777);
  int accepted = 0;
  for (int i = 0; i < 50000; ++i) {
    const Bytes f = random_wellformed_frame(rng);
    const auto r = decode(f.data(), f.size());
    if (r.ok()) {
      ++accepted;
      // 往返一致：decode 没有丢信息，encode 也没有加信息
      ASSERT_EQ(bytes_of(encode(r.frame.seq, r.frame.payload)), f);
    } else {
      ASSERT_EQ(r.error, DecodeError::BadPayloadSize);
    }
  }
  // 确认这个测试真的走到了"接受"的分支，而不是全部被拒绝而白跑
  EXPECT_GT(accepted, 1000);
}

// 性质 3：改动任意一个字节，必然被拒绝。CRC-16 能检出所有不超过 16 位的突发错误
TEST(ProtocolFuzz, AnySingleByteCorruptionIsRejected) {
  std::mt19937 rng(31337);
  for (int i = 0; i < 2000; ++i) {
    const Bytes original = random_wellformed_frame(rng);
    for (std::size_t pos = 0; pos < original.size(); ++pos) {
      Bytes damaged = original;
      damaged[pos] ^= static_cast<std::uint8_t>(1 + rng() % 255);   // 异或非零值 => 一定改变该字节
      ASSERT_FALSE(decode(damaged.data(), damaged.size()).ok())
          << "第 " << i << " 个帧，位置 " << pos;
    }
  }
}

// 性质 4：任意 float 位模式（含 NaN、无穷、非规格化数）往返后位完全一致
TEST(ProtocolFuzz, RandomFloatBitPatternsSurviveRoundTrip) {
  std::mt19937 rng(2468);
  for (int i = 0; i < 20000; ++i) {
    Telemetry t;
    t.x = float_of(static_cast<std::uint32_t>(rng()));
    t.y = float_of(static_cast<std::uint32_t>(rng()));
    t.heading = float_of(static_cast<std::uint32_t>(rng()));
    t.speed = float_of(static_cast<std::uint32_t>(rng()));
    t.state = static_cast<VehicleState>(rng() % 256);   // 包括枚举里不存在的值
    t.battery_pct = static_cast<std::uint8_t>(rng());

    const auto seq = static_cast<std::uint16_t>(rng());
    const auto enc = encode(seq, t);
    const auto res = decode(enc.data.data(), enc.size);
    ASSERT_TRUE(res.ok());
    ASSERT_EQ(res.frame.seq, seq);
    const auto& g = std::get<Telemetry>(res.frame.payload);
    ASSERT_EQ(bits_of(g.x), bits_of(t.x));
    ASSERT_EQ(bits_of(g.y), bits_of(t.y));
    ASSERT_EQ(bits_of(g.heading), bits_of(t.heading));
    ASSERT_EQ(bits_of(g.speed), bits_of(t.speed));
    ASSERT_EQ(g.state, t.state);
    ASSERT_EQ(g.battery_pct, t.battery_pct);
  }
}
