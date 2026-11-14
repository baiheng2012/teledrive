#include <gtest/gtest.h>
#include <teledrive/node/names.hpp>

#include <cstring>

using namespace teledrive;
using teledrive::node::name;

TEST(Names, VehicleStatesHaveReadableNames) {
  EXPECT_STREQ(name(protocol::VehicleState::Idle), "IDLE");
  EXPECT_STREQ(name(protocol::VehicleState::Armed), "ARMED");
  EXPECT_STREQ(name(protocol::VehicleState::Driving), "DRIVING");
  EXPECT_STREQ(name(protocol::VehicleState::Fault), "FAULT");
}

TEST(Names, FaultReasonsHaveReadableNames) {
  EXPECT_STREQ(name(safety::FaultReason::None), "-");
  EXPECT_STREQ(name(safety::FaultReason::EStop), "ESTOP");
  EXPECT_STREQ(name(safety::FaultReason::LinkTimeout), "LINK_TIMEOUT");
  EXPECT_STREQ(name(safety::FaultReason::InvalidCommand), "INVALID_CMD");
}

// 收到协议里不存在的状态值（比如被篡改但 CRC 碰巧正确）：不能崩溃，返回占位符
TEST(Names, UnknownValuesGiveAPlaceholder) {
  EXPECT_STREQ(name(static_cast<protocol::VehicleState>(200)), "?");
  EXPECT_STREQ(name(static_cast<safety::FaultReason>(200)), "?");
}
