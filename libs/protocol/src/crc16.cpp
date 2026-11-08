#include "teledrive/protocol/crc16.hpp"

namespace teledrive::protocol {

std::uint16_t crc16(const std::uint8_t* data, std::size_t len) {
  std::uint16_t crc = 0xFFFF;  // 初始值
  for (std::size_t i = 0; i < len; ++i) {
    // 把当前字节异或进 CRC 的高 8 位
    crc ^= static_cast<std::uint16_t>(data[i]) << 8;
    // 逐位处理：最高位为 1 就左移并异或多项式，否则只左移
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<std::uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<std::uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

}  // namespace teledrive::protocol
