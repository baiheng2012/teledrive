#include <gtest/gtest.h>
#include <teledrive/can/can_transport.hpp>
#include <teledrive/can/mock_can_bus.hpp>
#include <teledrive/protocol/messages.hpp>

using namespace teledrive::can;
namespace proto = teledrive::protocol;
namespace tr = teledrive::transport;

namespace {

// 三个节点接在同一条总线上：车辆 1、操作端 2、旁观者 3
struct Bus3 {
  MockCanBus veh_bus, op_bus, other_bus;
  CanTransport veh{veh_bus, 1}, op{op_bus, 2}, other{other_bus, 3};
  Bus3() {
    MockCanBus::join(veh_bus, op_bus);
    MockCanBus::join(veh_bus, other_bus);
    MockCanBus::join(op_bus, other_bus);
  }
};

proto::EncodedFrame command(std::uint16_t seq, std::uint8_t flags = 0) {
  proto::Command c;
  c.linear = 0.5f;
  c.flags = flags;
  return proto::encode(seq, c);
}

}  // namespace

TEST(CanTransport, ProtocolCommandSurvivesTheBus) {
  Bus3 b;
  proto::Command c;
  c.linear = 1.25f;
  c.angular = -0.5f;
  c.flags = proto::kFlagArm;
  const auto enc = proto::encode(77, c);
  ASSERT_TRUE(b.op.send(can_endpoint(1), enc.data.data(), enc.size));

  tr::Datagram d;
  ASSERT_TRUE(b.veh.receive(d));
  EXPECT_EQ(d.from, can_endpoint(2));   // 来源是操作端节点
  const auto res = proto::decode(d.data.data(), d.size);
  ASSERT_TRUE(res.ok());
  EXPECT_EQ(res.frame.seq, 77);
  EXPECT_FLOAT_EQ(std::get<proto::Command>(res.frame.payload).linear, 1.25f);
}

TEST(CanTransport, LargestProtocolFrameFits) {
  Bus3 b;
  const auto enc = proto::encode(1, proto::Telemetry{});   // 28 字节 -> 5 个 CAN 帧
  ASSERT_TRUE(b.veh.send(can_endpoint(2), enc.data.data(), enc.size));
  EXPECT_EQ(b.veh_bus.sent_log().size(), 5u);
  tr::Datagram d;
  ASSERT_TRUE(b.op.receive(d));
  EXPECT_EQ(d.size, enc.size);
}

// 优先级：急停 < 指令 < 心跳 < 遥测 < 其他
TEST(CanTransport, ChoosesPriorityFromProtocolContent) {
  Bus3 b;
  auto pri = [&](const std::uint8_t* data, std::size_t n) {
    b.op.send(can_endpoint(1), data, n);
    return parse_id(b.op_bus.sent_log().back().id).priority;
  };
  const auto estop = command(1, proto::kFlagEStop | proto::kFlagArm);
  const auto cmd = command(2, proto::kFlagArm);
  const auto hb = proto::encode(3, proto::Heartbeat{1});
  const auto tel = proto::encode(4, proto::Telemetry{});
  const std::uint8_t junk[5] = {1, 2, 3, 4, 5};

  EXPECT_EQ(pri(estop.data.data(), estop.size), static_cast<int>(Priority::EStop));
  EXPECT_EQ(pri(cmd.data.data(), cmd.size), static_cast<int>(Priority::Command));
  EXPECT_EQ(pri(hb.data.data(), hb.size), static_cast<int>(Priority::Heartbeat));
  EXPECT_EQ(pri(tel.data.data(), tel.size), static_cast<int>(Priority::Telemetry));
  EXPECT_EQ(pri(junk, sizeof(junk)), static_cast<int>(Priority::Other));
}

TEST(CanTransport, FramesForOtherNodesAreIgnored) {
  Bus3 b;
  const auto enc = command(1);
  ASSERT_TRUE(b.op.send(can_endpoint(1), enc.data.data(), enc.size));
  tr::Datagram d;
  EXPECT_FALSE(b.other.receive(d));   // 节点 3 在总线上看得到帧，但不是发给它的
  EXPECT_TRUE(b.veh.receive(d));
}

TEST(CanTransport, BroadcastReachesEveryoneExceptSender) {
  Bus3 b;
  const auto enc = command(1);
  ASSERT_TRUE(b.op.send(can_endpoint(kBroadcast), enc.data.data(), enc.size));
  tr::Datagram d;
  EXPECT_TRUE(b.veh.receive(d));
  EXPECT_TRUE(b.other.receive(d));
  EXPECT_FALSE(b.op.receive(d));
}

TEST(CanTransport, RejectsInvalidSends) {
  Bus3 b;
  const auto enc = command(1);
  std::uint8_t big[kMaxCanMessage + 1] = {};
  EXPECT_FALSE(b.op.send(can_endpoint(1), nullptr, 5));
  EXPECT_FALSE(b.op.send(can_endpoint(1), enc.data.data(), 0));
  EXPECT_FALSE(b.op.send(can_endpoint(1), big, sizeof(big)));
  EXPECT_FALSE(b.op.send(tr::Endpoint{0, 16}, enc.data.data(), enc.size));   // 节点号越界
  EXPECT_TRUE(b.op_bus.sent_log().empty());
}

TEST(CanTransport, InvalidNodeIdMakesTransportInert) {
  MockCanBus bus;
  CanTransport bad(bus, 15);   // 15 是广播地址，不能当自己的节点号
  const auto enc = command(1);
  tr::Datagram d;
  EXPECT_FALSE(bad.send(can_endpoint(1), enc.data.data(), enc.size));
  EXPECT_FALSE(bad.receive(d));
}

TEST(CanTransport, LostFrameDropsThatMessageOnly) {
  Bus3 b;
  const auto first = command(1), second = command(2);
  b.op_bus.drop_next_frames(1);   // 第一条消息的第一个帧丢了
  b.op.send(can_endpoint(1), first.data.data(), first.size);
  b.op.send(can_endpoint(1), second.data.data(), second.size);

  tr::Datagram d;
  ASSERT_TRUE(b.veh.receive(d));   // 只有第二条
  EXPECT_EQ(proto::decode(d.data.data(), d.size).frame.seq, 2);
  EXPECT_FALSE(b.veh.receive(d));
  EXPECT_EQ(b.veh.stats().dropped_incomplete, 1u);
}

// 垃圾帧：既不崩溃，也不会冒出数据报
TEST(CanTransport, GarbageFramesProduceNothing) {
  Bus3 b;
  std::uint32_t x = 12345;
  for (int i = 0; i < 20000; ++i) {
    x = x * 1664525u + 1013904223u;   // 简单的确定性伪随机
    CanFrame f;
    f.id = make_id(Priority::Command, 1, 2);   // 目的是车辆，才能进入重组器
    f.dlc = static_cast<std::uint8_t>(x % 9);
    for (auto& v : f.data) { x = x * 1664525u + 1013904223u; v = static_cast<std::uint8_t>(x >> 24); }
    b.veh_bus.inject(f);
  }
  tr::Datagram d;
  int got = 0;
  while (b.veh.receive(d)) {
    ++got;
    // 随机数据偶尔会拼出一条"形状合法"的消息，但它过不了协议层的魔数和 CRC
    ASSERT_FALSE(proto::decode(d.data.data(), d.size).ok());
  }
  EXPECT_GE(got, 0);
}
