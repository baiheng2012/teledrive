#include <gtest/gtest.h>
#include <teledrive/protocol/byte_io.hpp>
#include <teledrive/protocol/crc16.hpp>
#include <teledrive/transport/mock_transport.hpp>
#include <teledrive/video/video_packet.hpp>

#include <algorithm>

using namespace teledrive::video;
namespace proto = teledrive::protocol;
namespace tr = teledrive::transport;

namespace {

// 生成 n 字节的有规律数据，便于逐字节比对
std::vector<std::uint8_t> make_data(std::size_t n) {
  std::vector<std::uint8_t> d(n);
  for (std::size_t i = 0; i < n; ++i) d[i] = static_cast<std::uint8_t>(i * 7 + 3);
  return d;
}

std::vector<Packet> frag(std::uint16_t id, std::size_t n, std::uint32_t ms = 0) {
  const auto d = make_data(n);
  return fragment_frame(id, ms, d.data(), d.size());
}

// 改了包头字节之后，重新计算 CRC：这样才能测到"CRC 正确但内容非法"的分支
void fix_crc(Packet& p) {
  const std::size_t body = p.size() - kVideoCrcSize;
  proto::put_u16(p.data(), body, proto::crc16(p.data(), body));
}

constexpr std::size_t kOffVersion = 2, kOffCount = 3, kOffIndex = 4;

}  // namespace

// ---------- 分片 ----------

TEST(VideoFragment, SplitsAtPayloadBoundaries) {
  EXPECT_EQ(frag(1, 1).size(), 1u);
  EXPECT_EQ(frag(1, 1200).size(), 1u);
  EXPECT_EQ(frag(1, 1201).size(), 2u);
  EXPECT_EQ(frag(1, 5000).size(), 5u);
}

TEST(VideoFragment, RejectsEmptyAndOversizedFrames) {
  EXPECT_TRUE(fragment_frame(1, 0, nullptr, 10).empty());
  const auto d = make_data(10);
  EXPECT_TRUE(fragment_frame(1, 0, d.data(), 0).empty());
  EXPECT_TRUE(frag(1, kMaxFrameBytes + 1).empty());
  EXPECT_EQ(frag(1, kMaxFrameBytes).size(), kMaxFragments);
}

// ---------- 重组 ----------

TEST(VideoReassembler, RoundTripInOrder) {
  const auto data = make_data(5000);
  const auto packets = fragment_frame(42, 1234, data.data(), data.size());
  FrameReassembler r;
  VideoFrame f;
  for (std::size_t i = 0; i + 1 < packets.size(); ++i) {
    EXPECT_FALSE(r.push(packets[i].data(), packets[i].size(), f));   // 没收齐
  }
  ASSERT_TRUE(r.push(packets.back().data(), packets.back().size(), f));
  EXPECT_EQ(f.frame_id, 42);
  EXPECT_EQ(f.capture_ms, 1234u);
  EXPECT_EQ(f.data, data);
  EXPECT_EQ(r.stats().completed, 1u);
}

TEST(VideoReassembler, HandlesOutOfOrderArrival) {
  const auto data = make_data(4000);
  auto packets = fragment_frame(1, 0, data.data(), data.size());
  std::reverse(packets.begin(), packets.end());
  FrameReassembler r;
  VideoFrame f;
  bool done = false;
  for (const auto& p : packets) done = r.push(p.data(), p.size(), f);
  ASSERT_TRUE(done);
  EXPECT_EQ(f.data, data);
}

TEST(VideoReassembler, SingleFragmentFrame) {
  const auto packets = frag(9, 100);
  ASSERT_EQ(packets.size(), 1u);
  FrameReassembler r;
  VideoFrame f;
  EXPECT_TRUE(r.push(packets[0].data(), packets[0].size(), f));
  EXPECT_EQ(f.data.size(), 100u);
}

TEST(VideoReassembler, DuplicateFragmentIsIgnored) {
  const auto packets = frag(1, 2500);   // 3 片
  FrameReassembler r;
  VideoFrame f;
  r.push(packets[0].data(), packets[0].size(), f);
  EXPECT_FALSE(r.push(packets[0].data(), packets[0].size(), f));   // 重复
  EXPECT_EQ(r.stats().stale_packets, 1u);
  r.push(packets[1].data(), packets[1].size(), f);
  EXPECT_TRUE(r.push(packets[2].data(), packets[2].size(), f));    // 重复片没有干扰重组
}

// 丢了一片：这一帧作废；下一帧到来时被计入 dropped_incomplete，且不会交付残缺数据
TEST(VideoReassembler, LostFragmentDropsFrameWithoutDeliveringIt) {
  const auto f1 = frag(1, 3000);   // 3 片
  FrameReassembler r;
  VideoFrame out;
  EXPECT_FALSE(r.push(f1[0].data(), f1[0].size(), out));
  EXPECT_FALSE(r.push(f1[2].data(), f1[2].size(), out));   // 第 1 片丢了

  const auto f2 = frag(2, 100);
  ASSERT_TRUE(r.push(f2[0].data(), f2[0].size(), out));
  EXPECT_EQ(out.frame_id, 2);
  EXPECT_EQ(r.stats().dropped_incomplete, 1u);
  EXPECT_EQ(r.stats().completed, 1u);
}

// 旧帧的迟到分片：新帧已经开始，旧帧不能再插进来
TEST(VideoReassembler, LateFragmentOfOlderFrameIsStale) {
  const auto f1 = frag(1, 3000);
  const auto f2 = frag(2, 3000);
  FrameReassembler r;
  VideoFrame out;
  r.push(f1[0].data(), f1[0].size(), out);
  r.push(f2[0].data(), f2[0].size(), out);        // 新帧开始，旧帧被放弃
  EXPECT_FALSE(r.push(f1[1].data(), f1[1].size(), out));
  EXPECT_EQ(r.stats().stale_packets, 1u);
}

// 已完成帧的重放（网络重复）不会让同一帧被再交付一次
TEST(VideoReassembler, CompletedFrameIsNotDeliveredTwice) {
  const auto p = frag(7, 2000);   // 2 片
  FrameReassembler r;
  VideoFrame out;
  r.push(p[0].data(), p[0].size(), out);
  ASSERT_TRUE(r.push(p[1].data(), p[1].size(), out));
  EXPECT_FALSE(r.push(p[0].data(), p[0].size(), out));
  EXPECT_EQ(r.stats().stale_packets, 1u);
  EXPECT_EQ(r.stats().completed, 1u);
}

TEST(VideoReassembler, FrameIdWrapAroundIsAccepted) {
  const auto a = frag(65535, 100);
  const auto b = frag(0, 100);     // 65535 的下一帧
  FrameReassembler r;
  VideoFrame out;
  EXPECT_TRUE(r.push(a[0].data(), a[0].size(), out));
  EXPECT_TRUE(r.push(b[0].data(), b[0].size(), out));
  EXPECT_EQ(r.stats().completed, 2u);
}

// ---------- 坏包 ----------

namespace {
void expect_rejected(const Packet& p) {
  FrameReassembler r;
  VideoFrame out;
  EXPECT_FALSE(r.push(p.data(), p.size(), out));
  EXPECT_EQ(r.stats().bad_packets, 1u);
}
}  // namespace

TEST(VideoReassembler, RejectsCorruptedPayload) {
  auto p = frag(1, 100)[0];
  p[kVideoHeaderSize + 5] ^= 0x01;        // 改载荷里的一个比特，CRC 对不上
  expect_rejected(p);
}

TEST(VideoReassembler, RejectsMalformedHeaders) {
  const Packet good = frag(1, 100)[0];

  Packet p = good;  p[0] ^= 0xFF;            fix_crc(p);  expect_rejected(p);   // magic
  p = good;         p[kOffVersion] = 9;      fix_crc(p);  expect_rejected(p);   // 版本
  p = good;         p[kOffCount] = 0;        fix_crc(p);  expect_rejected(p);   // 总片数为 0
  p = good;         p[kOffIndex] = p[kOffCount]; fix_crc(p); expect_rejected(p); // 序号 >= 总数

  p = good;  p.pop_back();   expect_rejected(p);     // 截断
  p = good;  p.push_back(0); expect_rejected(p);     // 尾部多余字节

  FrameReassembler r;
  VideoFrame out;
  EXPECT_FALSE(r.push(nullptr, 0, out));
  EXPECT_EQ(r.stats().bad_packets, 1u);
}

// 同一个 frame_id 的分片声称不同的总片数：不可信，拒绝
TEST(VideoReassembler, RejectsInconsistentFragmentCountWithinOneFrame) {
  const auto three = frag(5, 3000);   // 3 片
  const auto four = frag(5, 4000);    // 4 片，同样的 frame_id
  FrameReassembler r;
  VideoFrame out;
  r.push(three[0].data(), three[0].size(), out);
  EXPECT_FALSE(r.push(four[1].data(), four[1].size(), out));
  EXPECT_EQ(r.stats().bad_packets, 1u);
}

// ---------- 经过 MockTransport：同时验证 1500 的数据报上限够用 ----------

TEST(VideoOverTransport, FragmentsFitInDatagramsAndReassemble) {
  const tr::Endpoint a{0x7F000001u, 1}, b{0x7F000001u, 2};
  tr::MockTransport tx(a), rx(b);
  tx.connect(&rx);

  const auto data = make_data(20000);
  for (const auto& p : fragment_frame(3, 77, data.data(), data.size())) {
    ASSERT_TRUE(tx.send(b, p.data(), p.size())) << "分片超过了数据报上限";
  }

  FrameReassembler r;
  VideoFrame out;
  tr::Datagram d;
  bool done = false;
  while (rx.receive(d)) done = r.push(d.data.data(), d.size, out) || done;
  ASSERT_TRUE(done);
  EXPECT_EQ(out.data, data);
}
