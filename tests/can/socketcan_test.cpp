// 真实 SocketCAN 测试，需要 vcan0 接口：
//   sudo ip link add dev vcan0 type vcan; sudo ip link set up vcan0
// 接口不存在时测试被跳过（SKIPPED，不是通过）；设置环境变量 TELEDRIVE_REQUIRE_VCAN=1 则改为失败，
// 用于 CI，避免"悄悄跳过还显示绿色"。
#include <gtest/gtest.h>
#include <teledrive/can/can_transport.hpp>
#include <teledrive/can/socketcan_bus.hpp>
#include <teledrive/protocol/messages.hpp>

#include <chrono>
#include <cstdlib>
#include <thread>

using namespace teledrive::can;
using namespace std::chrono_literals;
namespace proto = teledrive::protocol;
namespace tr = teledrive::transport;

namespace {

class RealVcan : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string err;
    a_ = SocketCanBus::open("vcan0", &err);
    b_ = SocketCanBus::open("vcan0", &err);
    if (!a_ || !b_) {
      if (std::getenv("TELEDRIVE_REQUIRE_VCAN") != nullptr) FAIL() << "vcan0 unavailable: " << err;
      GTEST_SKIP() << "vcan0 unavailable: " << err;
    }
    CanFrame drop;
    while (a_->receive_frame(drop)) {}   // 清掉接口上残留的帧
    while (b_->receive_frame(drop)) {}
  }

  static bool wait_frame(ICanBus& bus, CanFrame& f) {
    const auto deadline = std::chrono::steady_clock::now() + 1000ms;
    while (std::chrono::steady_clock::now() < deadline) {
      if (bus.receive_frame(f)) return true;
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  std::unique_ptr<SocketCanBus> a_, b_;
};

}  // namespace

TEST_F(RealVcan, FrameCrossesTheBusAndSenderDoesNotHearItself) {
  CanFrame f;
  f.id = 0x123;
  f.dlc = 3;
  f.data = {0xDE, 0xAD, 0xBE};
  ASSERT_TRUE(a_->send_frame(f));

  CanFrame got;
  ASSERT_TRUE(wait_frame(*b_, got));
  EXPECT_EQ(got.id, 0x123u);
  EXPECT_EQ(got.dlc, 3);
  EXPECT_EQ(got.data[0], 0xDE);
  EXPECT_FALSE(a_->receive_frame(got));   // 默认不接收自己发的帧
}

TEST_F(RealVcan, ReceiveIsNonBlocking) {
  CanFrame f;
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(a_->receive_frame(f));
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 100ms);
}

TEST_F(RealVcan, RejectsInvalidFrames) {
  CanFrame f;
  f.id = 0x800;   // 超过 11 位
  EXPECT_FALSE(a_->send_frame(f));
  f.id = 0x100;
  f.dlc = 9;
  EXPECT_FALSE(a_->send_frame(f));
}

TEST(SocketCan, OpeningMissingInterfaceFailsCleanly) {
  std::string err;
  EXPECT_EQ(SocketCanBus::open("nocan99", &err), nullptr);
  EXPECT_FALSE(err.empty());
}

// 协议帧经过真实 CAN 接口：分片、发送、重组、协议层解码
TEST_F(RealVcan, ProtocolFrameOverRealCan) {
  CanTransport ta(*a_, 2), tb(*b_, 1);
  proto::Command c;
  c.linear = 0.75f;
  c.flags = proto::kFlagArm;
  const auto enc = proto::encode(5, c);
  ASSERT_TRUE(ta.send(can_endpoint(1), enc.data.data(), enc.size));

  tr::Datagram d;
  bool got = false;
  const auto deadline = std::chrono::steady_clock::now() + 1000ms;
  while (!got && std::chrono::steady_clock::now() < deadline) {
    got = tb.receive(d);
    if (!got) std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(got);
  const auto res = proto::decode(d.data.data(), d.size);
  ASSERT_TRUE(res.ok());
  EXPECT_FLOAT_EQ(std::get<proto::Command>(res.frame.payload).linear, 0.75f);
}
