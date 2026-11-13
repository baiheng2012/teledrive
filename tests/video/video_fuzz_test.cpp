// 视频重组器的模糊测试：模拟一条糟糕的网络（丢包、重复、乱序、比特损坏）。
#include <gtest/gtest.h>
#include <teledrive/video/video_packet.hpp>

#include <algorithm>
#include <map>
#include <random>
#include <vector>

using namespace teledrive::video;

// 随机字节永远不应被接受，也不应崩溃
TEST(VideoFuzz, RandomBytesAreNeverAccepted) {
  std::mt19937 rng(7);
  FrameReassembler r;
  VideoFrame out;
  std::vector<std::uint8_t> buf(1600);
  for (int i = 0; i < 100000; ++i) {
    const std::size_t len = rng() % buf.size();
    for (std::size_t j = 0; j < len; ++j) buf[j] = static_cast<std::uint8_t>(rng());
    ASSERT_FALSE(r.push(buf.data(), len, out));
  }
  EXPECT_EQ(r.stats().completed, 0u);
}

// 核心性质：被交付的每一帧，内容都与原始数据完全一致，且帧号严格递增。
// 网络条件：10% 丢包，5% 比特损坏，10% 重复，相邻几个包乱序。
TEST(VideoFuzz, DamagedStreamNeverDeliversCorruptOrReorderedFrames) {
  std::mt19937 rng(99);
  constexpr int kFrames = 300;

  std::map<std::uint16_t, std::vector<std::uint8_t>> originals;   // 帧号 -> 原始 JPEG 数据
  std::vector<Packet> stream;                                     // 经过"糟糕网络"之后的包序列

  for (int id = 0; id < kFrames; ++id) {
    std::vector<std::uint8_t> data(1 + rng() % 20000);   // 1 ~ 17 个分片
    for (auto& b : data) b = static_cast<std::uint8_t>(rng());
    originals[static_cast<std::uint16_t>(id)] = data;

    for (auto& p : fragment_frame(static_cast<std::uint16_t>(id),
                                  static_cast<std::uint32_t>(id * 10), data.data(), data.size())) {
      const unsigned roll = rng() % 100;
      if (roll < 10) continue;                                   // 丢包
      if (roll < 15) {                                           // 损坏：改一个字节
        p[rng() % p.size()] ^= static_cast<std::uint8_t>(1 + rng() % 255);
      }
      stream.push_back(p);
      if (roll >= 15 && roll < 25) stream.push_back(p);          // 重复
    }
  }

  // 局部乱序：每个包与它后面 0~3 个位置之内的包交换
  for (std::size_t i = 0; i < stream.size(); ++i) {
    const std::size_t j = i + rng() % 4;
    if (j < stream.size()) std::swap(stream[i], stream[j]);
  }

  FrameReassembler r;
  VideoFrame out;
  int delivered = 0;
  bool have_prev = false;
  std::uint16_t prev = 0;
  for (const auto& p : stream) {
    if (!r.push(p.data(), p.size(), out)) continue;
    ++delivered;
    ASSERT_EQ(out.data, originals.at(out.frame_id)) << "交付了内容不一致的帧 " << out.frame_id;
    ASSERT_EQ(out.capture_ms, static_cast<std::uint32_t>(out.frame_id * 10));
    if (have_prev) {ASSERT_TRUE(frame_id_newer(out.frame_id, prev)) << "帧号没有递增";}
    have_prev = true;
    prev = out.frame_id;
  }

  // 确认测试真的收到过完整的帧（避免"全部被丢弃所以没出错"的假通过）
  EXPECT_GT(delivered, 10);
  EXPECT_GT(r.stats().bad_packets, 0u);        // 损坏的包确实被识别了
  EXPECT_GT(r.stats().stale_packets, 0u);      // 重复/过期的包确实被识别了
}
