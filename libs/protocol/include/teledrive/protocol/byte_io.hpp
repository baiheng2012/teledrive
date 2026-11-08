#pragma once
// 小端字节序的读写工具。
// 思路：逐字节用移位运算拼装，不依赖机器本身的字节序，
// 所以在大端或小端机器上结果都一样（可移植）。

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace teledrive::protocol {

// ---- 写入 ----
// 所有 put 函数：把值写到 buf 的 offset 位置，并返回写入后的新 offset，
// 这样可以链式调用：off = put_u16(buf, off, a); off = put_u32(buf, off, b);

inline std::size_t put_u8(std::uint8_t* buf, std::size_t off, std::uint8_t v) {
  buf[off] = v;
  return off + 1;
}

inline std::size_t put_u16(std::uint8_t* buf, std::size_t off, std::uint16_t v) {
  buf[off]     = static_cast<std::uint8_t>(v & 0xFF);         // 低字节在前（小端）
  buf[off + 1] = static_cast<std::uint8_t>((v >> 8) & 0xFF);  // 高字节在后
  return off + 2;
}

inline std::size_t put_u32(std::uint8_t* buf, std::size_t off, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    buf[off + i] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF);
  }
  return off + 4;
}

inline std::size_t put_f32(std::uint8_t* buf, std::size_t off, float v) {
  static_assert(sizeof(float) == 4, "protocol assumes 32-bit float");
  std::uint32_t bits;
  // memcpy 是 C++17 下把 float 的位模式取出来的合法方式
  // （直接 reinterpret_cast 会违反严格别名规则，属于未定义行为）。编译器会把它优化成一条指令。
  std::memcpy(&bits, &v, sizeof(bits));
  return put_u32(buf, off, bits);
}

// ---- 读取 ----
// 调用者必须保证 buf 里有足够的字节（长度检查在 decode 函数里统一做）。

inline std::uint8_t get_u8(const std::uint8_t* buf, std::size_t off) {
  return buf[off];
}

inline std::uint16_t get_u16(const std::uint8_t* buf, std::size_t off) {
  return static_cast<std::uint16_t>(buf[off] | (buf[off + 1] << 8));
}

inline std::uint32_t get_u32(const std::uint8_t* buf, std::size_t off) {
  std::uint32_t v = 0;
  for (int i = 0; i < 4; ++i) {
    v |= static_cast<std::uint32_t>(buf[off + i]) << (8 * i);
  }
  return v;
}

inline float get_f32(const std::uint8_t* buf, std::size_t off) {
  const std::uint32_t bits = get_u32(buf, off);
  float v;
  std::memcpy(&v, &bits, sizeof(v));
  return v;
}

}  // namespace teledrive::protocol
