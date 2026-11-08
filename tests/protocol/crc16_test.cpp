#include <gtest/gtest.h>
#include <teledrive/protocol/crc16.hpp>

#include <cstring>

using teledrive::protocol::crc16;

// 标准测试向量：CRC-16/CCITT-FALSE 对 "123456789" 的结果是 0x29B1。
// 这个值是业界公认的，能证明我们的实现和标准一致。
TEST(Crc16, StandardCheckValue) {
  const char* s = "123456789";
  EXPECT_EQ(crc16(reinterpret_cast<const std::uint8_t*>(s), std::strlen(s)), 0x29B1);
}

// 空数据：结果就是初始值
TEST(Crc16, EmptyInputReturnsInitValue) {
  EXPECT_EQ(crc16(nullptr, 0), 0xFFFF);
}

// 改变任意一个比特，CRC 必须变化
TEST(Crc16, SingleBitFlipChangesCrc) {
  std::uint8_t a[4] = {1, 2, 3, 4};
  std::uint8_t b[4] = {1, 2, 3, 4};
  b[2] ^= 0x01;
  EXPECT_NE(crc16(a, 4), crc16(b, 4));
}
