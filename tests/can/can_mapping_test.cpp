#include <gtest/gtest.h>
#include <teledrive/can/can_mapping.hpp>

#include <algorithm>
#include <random>
#include <vector>

using namespace teledrive::can;

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes make_msg(std::size_t n, std::uint8_t salt = 0) {
  Bytes b(n);
  for (std::size_t i = 0; i < n; ++i) b[i] = static_cast<std::uint8_t>(i * 13 + salt + 1);
  return b;
}

std::vector<CanFrame> frames_of(const Bytes& msg, std::uint8_t seq, std::uint32_t id = 0x123) {
  const auto m = fragment_message(id, seq, msg.data(), msg.size());
  return {m.frames.begin(), m.frames.begin() + m.count};
}

// 喂一帧；完成时把内容放进 out
bool feed(CanReassembler& r, std::uint8_t src, const CanFrame& f, Bytes& out) {
  std::uint8_t buf[kMaxCanMessage];
  std::size_t len = 0;
  if (!r.push(src, f, buf, len)) return false;
  out.assign(buf, buf + len);
  return true;
}

// 手工构造一个任意头部的帧，用来测非法输入
CanFrame raw_frame(std::uint8_t seq, std::uint8_t idx, std::uint8_t total, std::uint8_t payload_len) {
  CanFrame f;
  f.dlc = static_cast<std::uint8_t>(kFragHeader + payload_len);
  f.data[0] = static_cast<std::uint8_t>((seq << 4) | idx);
  f.data[1] = total;
  return f;
}

}  // namespace

// ---------- ID 与优先级 ----------

TEST(CanId, RoundTrip) {
  const auto id = make_id(Priority::Command, 3, 5);
  const auto f = parse_id(id);
  EXPECT_EQ(f.priority, 1);
  EXPECT_EQ(f.dst, 3);
  EXPECT_EQ(f.src, 5);
  EXPECT_LE(id, kMaxStandardId);
}

// ID 越小仲裁越优先：这条测试把"急停 > 指令 > 心跳 > 遥测"钉死
TEST(CanId, PriorityOrderIsArbitrationOrder) {
  EXPECT_LT(make_id(Priority::EStop, 1, 2), make_id(Priority::Command, 1, 2));
  EXPECT_LT(make_id(Priority::Command, 1, 2), make_id(Priority::Heartbeat, 1, 2));
  EXPECT_LT(make_id(Priority::Heartbeat, 1, 2), make_id(Priority::Telemetry, 1, 2));
  EXPECT_LT(make_id(Priority::Telemetry, 1, 2), make_id(Priority::Other, 1, 2));
  // 优先级在高位：即使节点号最大，急停也赢过节点号最小的普通指令
  EXPECT_LT(make_id(Priority::EStop, 15, 15), make_id(Priority::Command, 0, 0));
}

// ---------- 分片 ----------

TEST(CanFragment, FrameCountAtBoundaries) {
  EXPECT_EQ(frames_of(make_msg(1), 0).size(), 1u);
  EXPECT_EQ(frames_of(make_msg(6), 0).size(), 1u);
  EXPECT_EQ(frames_of(make_msg(7), 0).size(), 2u);
  EXPECT_EQ(frames_of(make_msg(96), 0).size(), 16u);
}

TEST(CanFragment, RejectsEmptyAndOversized) {
  EXPECT_EQ(fragment_message(1, 0, nullptr, 5).count, 0u);
  const auto m = make_msg(10);
  EXPECT_EQ(fragment_message(1, 0, m.data(), 0).count, 0u);
  const auto big = make_msg(97);
  EXPECT_EQ(fragment_message(1, 0, big.data(), big.size()).count, 0u);
}

TEST(CanFragment, FrameLayoutMatchesSpec) {
  const auto f = frames_of(make_msg(8), 5, 0x2A);   // 8 字节 -> 6 + 2
  ASSERT_EQ(f.size(), 2u);
  EXPECT_EQ(f[0].id, 0x2Au);
  EXPECT_EQ(f[0].dlc, 8);
  EXPECT_EQ(f[0].data[0], 0x50);   // 序号 5，片 0
  EXPECT_EQ(f[0].data[1], 2);      // 共 2 片
  EXPECT_EQ(f[1].dlc, 4);          // 头 2 字节 + 载荷 2 字节
  EXPECT_EQ(f[1].data[0], 0x51);   // 序号 5，片 1
}

// ---------- 重组 ----------

TEST(CanReassemble, EveryLengthRoundTrips) {
  for (std::size_t len = 1; len <= kMaxCanMessage; ++len) {
    CanReassembler r;
    const auto msg = make_msg(len);
    const auto frames = frames_of(msg, 3);
    Bytes out;
    for (std::size_t i = 0; i < frames.size(); ++i) {
      const bool done = feed(r, 1, frames[i], out);
      ASSERT_EQ(done, i + 1 == frames.size()) << "len=" << len << " i=" << i;
    }
    ASSERT_EQ(out, msg) << "len=" << len;
  }
}

TEST(CanReassemble, OutOfOrderFramesStillComplete) {
  CanReassembler r;
  const auto msg = make_msg(40);
  auto frames = frames_of(msg, 1);
  std::reverse(frames.begin(), frames.end());   // 末片最先到
  Bytes out;
  bool done = false;
  for (const auto& f : frames) done = feed(r, 1, f, out);
  ASSERT_TRUE(done);
  EXPECT_EQ(out, msg);
}

TEST(CanReassemble, DuplicateFrameIsIgnored) {
  CanReassembler r;
  const auto frames = frames_of(make_msg(20), 1);   // 4 片
  Bytes out;
  feed(r, 1, frames[0], out);
  EXPECT_FALSE(feed(r, 1, frames[0], out));
  EXPECT_EQ(r.stats().stale_frames, 1u);
}

TEST(CanReassemble, CompletedMessageIsNotDeliveredTwice) {
  CanReassembler r;
  const auto frames = frames_of(make_msg(20), 4);
  Bytes out;
  for (const auto& f : frames) feed(r, 1, f, out);
  ASSERT_EQ(r.stats().completed, 1u);
  for (const auto& f : frames) EXPECT_FALSE(feed(r, 1, f, out));   // 整条重放
  EXPECT_EQ(r.stats().completed, 1u);
}

TEST(CanReassemble, LostFrameDropsMessageAndNextOneRecovers) {
  CanReassembler r;
  const auto first = frames_of(make_msg(20, 1), 1);
  const auto second_msg = make_msg(20, 2);
  const auto second = frames_of(second_msg, 2);
  Bytes out;
  feed(r, 1, first[0], out);
  feed(r, 1, first[1], out);
  feed(r, 1, first[3], out);   // 第 2 片丢了

  bool done = false;
  for (const auto& f : second) done = feed(r, 1, f, out);
  ASSERT_TRUE(done);
  EXPECT_EQ(out, second_msg);
  EXPECT_EQ(r.stats().dropped_incomplete, 1u);
  EXPECT_EQ(r.stats().completed, 1u);   // 残缺的那条没有被交付
}

TEST(CanReassemble, SourcesAreIndependent) {
  CanReassembler r;
  const auto a = make_msg(30, 1), b = make_msg(25, 2);
  const auto fa = frames_of(a, 1), fb = frames_of(b, 1);   // 序号相同也不会串
  Bytes out, got_a, got_b;
  for (std::size_t i = 0; i < std::max(fa.size(), fb.size()); ++i) {   // 交错到达
    if (i < fa.size() && feed(r, 1, fa[i], out)) got_a = out;
    if (i < fb.size() && feed(r, 2, fb[i], out)) got_b = out;
  }
  EXPECT_EQ(got_a, a);
  EXPECT_EQ(got_b, b);
}

TEST(CanReassemble, MalformedFramesAreRejected) {
  CanReassembler r;
  Bytes out;
  CanFrame f;
  f.dlc = 1;                                   EXPECT_FALSE(feed(r, 1, f, out));   // 连头都放不下
  f.dlc = 9;                                   EXPECT_FALSE(feed(r, 1, f, out));   // 超过 8
  EXPECT_FALSE(feed(r, 1, raw_frame(0, 0, 0, 3), out));    // 总片数 0
  EXPECT_FALSE(feed(r, 1, raw_frame(0, 0, 17, 3), out));   // 总片数 17
  EXPECT_FALSE(feed(r, 1, raw_frame(0, 3, 3, 3), out));    // 序号 >= 总数
  EXPECT_FALSE(feed(r, 1, raw_frame(0, 0, 3, 5), out));    // 非末片不是 6 字节
  EXPECT_FALSE(feed(r, 1, raw_frame(0, 2, 3, 0), out));    // 末片没有载荷
  EXPECT_FALSE(feed(r, 16, raw_frame(0, 0, 1, 3), out));   // 源节点号越界
  EXPECT_EQ(r.stats().bad_frames, 8u);
}

TEST(CanReassemble, InconsistentTotalWithinOneMessageIsRejected) {
  CanReassembler r;
  Bytes out;
  EXPECT_FALSE(feed(r, 1, raw_frame(2, 0, 3, 6), out));
  EXPECT_FALSE(feed(r, 1, raw_frame(2, 1, 4, 6), out));   // 同一序号，总片数变了
  EXPECT_EQ(r.stats().bad_frames, 1u);
}

// ---------- 随机测试（固定种子） ----------

// 任意随机帧：不崩溃；凡是交付的消息，长度都在 1..96
TEST(CanFuzz, RandomFramesNeverCrashAndLengthsAreSane) {
  std::mt19937 rng(4242);
  CanReassembler r;
  std::uint8_t buf[kMaxCanMessage];
  for (int i = 0; i < 200000; ++i) {
    CanFrame f;
    f.dlc = static_cast<std::uint8_t>(rng() % 10);   // 包含非法的 9
    for (auto& b : f.data) b = static_cast<std::uint8_t>(rng());
    std::size_t len = 0;
    if (r.push(static_cast<std::uint8_t>(rng() % 17), f, buf, len)) {
      ASSERT_GE(len, 1u);
      ASSERT_LE(len, kMaxCanMessage);
    }
  }
}

// 核心性质：片内乱序、偶尔丢一片。交付的消息必须与原文一致；没丢片的必须交付
TEST(CanFuzz, ShuffledAndLossyMessagesNeverDeliverWrongData) {
  std::mt19937 rng(99);
  CanReassembler r;
  int delivered = 0;
  for (int i = 0; i < 5000; ++i) {
    const auto msg = make_msg(1 + rng() % kMaxCanMessage, static_cast<std::uint8_t>(i));
    auto frames = frames_of(msg, static_cast<std::uint8_t>(i & 0x0F));
    const bool lose = (rng() % 100) < 30;
    if (lose) frames.erase(frames.begin() + rng() % frames.size());
    std::shuffle(frames.begin(), frames.end(), rng);

    Bytes out;
    int completions = 0;
    for (const auto& f : frames) {
      if (feed(r, 1, f, out)) {
        ++completions;
        ASSERT_EQ(out, msg) << "第 " << i << " 条消息内容错误";
      }
    }
    ASSERT_EQ(completions, lose ? 0 : 1) << "第 " << i << " 条消息";
    delivered += completions;
  }
  EXPECT_GT(delivered, 3000);
}
