#pragma once
#include <cstddef>
#include <cstdint>

namespace teledrive::protocol {

// CRC-16/CCITT-FALSE：poly=0x1021, init=0xFFFF, 不反转, xorout=0
// 用于检测传输中的比特错误（不是加密，也不防恶意篡改）。
std::uint16_t crc16(const std::uint8_t* data, std::size_t len);

}  // namespace teledrive::protocol
