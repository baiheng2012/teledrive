#include <gtest/gtest.h>
#include <teledrive/transport/endpoint.hpp>

using namespace teledrive::transport;

TEST(Endpoint, ParsesValidAddress) {
  auto e = make_endpoint("127.0.0.1", 9000);
  ASSERT_TRUE(e.has_value());
  EXPECT_EQ(e->ip, 0x7F000001u);  // 127.0.0.1，主机字节序
  EXPECT_EQ(e->port, 9000);
}

TEST(Endpoint, RejectsInvalidAddress) {
  EXPECT_FALSE(make_endpoint("999.1.1.1", 1).has_value());   // 段超出范围
  EXPECT_FALSE(make_endpoint("not an ip", 1).has_value());   // 不是数字
  EXPECT_FALSE(make_endpoint("", 1).has_value());            // 空串
}

TEST(Endpoint, ToStringRoundTrip) {
  auto e = make_endpoint("192.168.1.20", 5005);
  ASSERT_TRUE(e.has_value());
  EXPECT_EQ(to_string(*e), "192.168.1.20:5005");
}
