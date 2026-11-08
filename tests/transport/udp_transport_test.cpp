#include <gtest/gtest.h>
#include <teledrive/protocol/messages.hpp>
#include <teledrive/transport/udp_transport.hpp>

#include <chrono>
#include <thread>

using namespace teledrive::transport;
namespace proto = teledrive::protocol;
using namespace std::chrono_literals;

namespace {

// 真实网络有微小延迟，所以不能 send 完立刻断言收到。
// 这个工具在超时时间内轮询，收到就返回 true。
// 注意：超时只是"防止测试卡死"的上限，正常情况下几毫秒就返回。
bool receive_with_timeout(ITransport& t, Datagram& d, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (t.receive(d)) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return false;
}

// 本机回环地址 + 指定端口
Endpoint loopback(std::uint16_t port) { return *make_endpoint("127.0.0.1", port); }

}  // namespace

TEST(UdpTransport, OpenWithPortZeroGetsAssignedPort) {
  std::string err;
  auto t = UdpTransport::open(0, &err);
  ASSERT_NE(t, nullptr) << err;
  EXPECT_NE(t->local_port(), 0);  // 系统分配了一个真实端口
}

TEST(UdpTransport, SendsAndReceivesOverLoopback) {
  auto a = UdpTransport::open(0);
  auto b = UdpTransport::open(0);
  ASSERT_NE(a, nullptr);
  ASSERT_NE(b, nullptr);

  const std::uint8_t msg[4] = {0xDE, 0xAD, 0xBE, 0xEF};
  ASSERT_TRUE(a->send(loopback(b->local_port()), msg, 4));

  Datagram d;
  ASSERT_TRUE(receive_with_timeout(*b, d, 1000ms));
  EXPECT_EQ(d.size, 4u);
  EXPECT_EQ(d.data[0], 0xDE);
  EXPECT_EQ(d.data[3], 0xEF);
  EXPECT_EQ(d.from.port, a->local_port());  // 来源端口就是 a 的端口
}

// 非阻塞性：没数据时必须立刻返回。如果误写成阻塞模式，这个测试会卡住
TEST(UdpTransport, ReceiveIsNonBlocking) {
  auto t = UdpTransport::open(0);
  ASSERT_NE(t, nullptr);
  Datagram d;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(t->receive(d));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
}

TEST(UdpTransport, BindingSamePortTwiceFails) {
  auto a = UdpTransport::open(0);
  ASSERT_NE(a, nullptr);
  std::string err;
  auto b = UdpTransport::open(a->local_port(), &err);  // 端口已被 a 占用
  EXPECT_EQ(b, nullptr);
  EXPECT_FALSE(err.empty());  // 错误信息应说明原因（Address already in use）
}

// RAII：对象销毁后端口被释放，可以被重新绑定
TEST(UdpTransport, DestructorReleasesPort) {
  std::uint16_t port = 0;
  {
    auto a = UdpTransport::open(0);
    ASSERT_NE(a, nullptr);
    port = a->local_port();
  }  // a 在此析构，socket 被关闭
  auto b = UdpTransport::open(port);
  EXPECT_NE(b, nullptr);
}

TEST(UdpTransport, MovedFromObjectIsInertAndTargetWorks) {
  auto a = UdpTransport::open(0);
  auto peer = UdpTransport::open(0);
  ASSERT_NE(a, nullptr);
  ASSERT_NE(peer, nullptr);

  UdpTransport moved(std::move(*a));  // 所有权转移给 moved
  const std::uint8_t byte = 1;
  EXPECT_FALSE(a->send(loopback(peer->local_port()), &byte, 1));   // 被移走的对象不能再用
  EXPECT_TRUE(moved.send(loopback(peer->local_port()), &byte, 1)); // 新主人正常工作
}

TEST(UdpTransport, RejectsOversizedSend) {
  auto a = UdpTransport::open(0);
  ASSERT_NE(a, nullptr);
  std::uint8_t big[kMaxDatagramSize + 1] = {};
  EXPECT_FALSE(a->send(loopback(a->local_port()), big, sizeof(big)));
}

// 端到端：协议帧经过真实 UDP，仍能被正确解码
TEST(UdpTransport, ProtocolFrameOverRealUdp) {
  auto op = UdpTransport::open(0);
  auto veh = UdpTransport::open(0);
  ASSERT_NE(op, nullptr);
  ASSERT_NE(veh, nullptr);

  proto::Command cmd;
  cmd.linear = 0.5f;
  cmd.flags = proto::kFlagArm;
  auto enc = proto::encode(9, cmd);
  ASSERT_TRUE(op->send(loopback(veh->local_port()), enc.data.data(), enc.size));

  Datagram d;
  ASSERT_TRUE(receive_with_timeout(*veh, d, 1000ms));
  auto res = proto::decode(d.data.data(), d.size);
  ASSERT_TRUE(res.ok());
  EXPECT_EQ(res.frame.seq, 9);
  EXPECT_EQ(std::get<proto::Command>(res.frame.payload).flags, proto::kFlagArm);
}
