#include <gtest/gtest.h>
#include <teledrive/protocol/messages.hpp>
#include <teledrive/transport/mock_transport.hpp>

using namespace teledrive::transport;
namespace proto = teledrive::protocol;

namespace {
// 两个测试用地址
const Endpoint kOperator{0x7F000001u, 9000};
const Endpoint kVehicle{0x7F000001u, 9001};
const std::uint8_t kBytes[3] = {1, 2, 3};
}  // namespace

TEST(MockTransport, DeliversToPeerWithSourceAddress) {
  MockTransport op(kOperator), veh(kVehicle);
  op.connect(&veh);

  ASSERT_TRUE(op.send(kVehicle, kBytes, 3));

  Datagram d;
  ASSERT_TRUE(veh.receive(d));
  EXPECT_EQ(d.size, 3u);
  EXPECT_EQ(d.data[0], 1);
  EXPECT_EQ(d.data[2], 3);
  EXPECT_EQ(d.from, kOperator);  // 收到的包里记录了发送方
}

TEST(MockTransport, ReceiveOnEmptyReturnsFalseImmediately) {
  MockTransport t(kVehicle);
  Datagram d;
  EXPECT_FALSE(t.receive(d));
}

TEST(MockTransport, PreservesOrder) {
  MockTransport op(kOperator), veh(kVehicle);
  op.connect(&veh);
  for (std::uint8_t i = 0; i < 5; ++i) {
    op.send(kVehicle, &i, 1);
  }
  Datagram d;
  for (std::uint8_t i = 0; i < 5; ++i) {
    ASSERT_TRUE(veh.receive(d));
    EXPECT_EQ(d.data[0], i);  // 先发先到
  }
}

TEST(MockTransport, DropsPacketsOnDemand) {
  MockTransport op(kOperator), veh(kVehicle);
  op.connect(&veh);
  op.drop_next_sends(2);

  EXPECT_TRUE(op.send(kVehicle, kBytes, 3));  // 被丢，但发送方不知道
  EXPECT_TRUE(op.send(kVehicle, kBytes, 3));  // 被丢
  EXPECT_TRUE(op.send(kVehicle, kBytes, 3));  // 正常送达

  Datagram d;
  EXPECT_TRUE(veh.receive(d));    // 只有第三个到了
  EXPECT_FALSE(veh.receive(d));
  EXPECT_EQ(op.sent_count(), 3u);
}

TEST(MockTransport, RejectsInvalidSends) {
  MockTransport op(kOperator), veh(kVehicle);
  op.connect(&veh);
  EXPECT_FALSE(op.send(kVehicle, nullptr, 3));
  EXPECT_FALSE(op.send(kVehicle, kBytes, 0));
  std::uint8_t big[kMaxDatagramSize + 1] = {};
  EXPECT_FALSE(op.send(kVehicle, big, sizeof(big)));
}

TEST(MockTransport, SendWithoutPeerFails) {
  MockTransport lonely(kOperator);
  EXPECT_FALSE(lonely.send(kVehicle, kBytes, 3));
}

// 协议层 + 传输层配合：Command 经过"网络"后能被完整还原。
// 这是后面车辆端接收逻辑的雏形。
TEST(MockTransport, ProtocolFrameSurvivesTransport) {
  MockTransport op(kOperator), veh(kVehicle);
  op.connect(&veh);

  proto::Command cmd;
  cmd.linear = 1.2f;
  cmd.angular = -0.3f;
  auto enc = proto::encode(42, cmd);
  ASSERT_TRUE(op.send(kVehicle, enc.data.data(), enc.size));

  Datagram d;
  ASSERT_TRUE(veh.receive(d));
  auto res = proto::decode(d.data.data(), d.size);
  ASSERT_TRUE(res.ok());
  EXPECT_EQ(res.frame.seq, 42);
  EXPECT_FLOAT_EQ(std::get<proto::Command>(res.frame.payload).linear, 1.2f);
}

// 注入损坏的数据：传输层照常交付，由协议层的 CRC 拒绝。
// 证明两层职责分离：transport 不判断内容对不对。
TEST(MockTransport, CorruptedBytesAreRejectedByProtocolLayer) {
  MockTransport veh(kVehicle);
  auto enc = proto::encode(1, proto::Command{});
  enc.data[10] ^= 0xFF;  // 篡改载荷
  veh.inject(kOperator, enc.data.data(), enc.size);

  Datagram d;
  ASSERT_TRUE(veh.receive(d));  // 传输层：照常收到
  EXPECT_EQ(proto::decode(d.data.data(), d.size).error, proto::DecodeError::BadCrc);
}
