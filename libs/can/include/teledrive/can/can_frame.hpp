#pragma once
// 经典 CAN 帧（11 位标准 ID，最多 8 字节数据）与总线接口。
// 把"收发一个原始帧"抽象成 ICanBus：
//   - 真实实现：SocketCanBus（Linux SocketCAN）
//   - 测试实现：MockCanBus（内存，可注入丢帧）
// 这样分片、重组、优先级这些逻辑不需要真实的 CAN 接口就能测试。

#include <array>
#include <cstddef>
#include <cstdint>

namespace teledrive::can {

inline constexpr std::size_t kMaxDlc = 8;
inline constexpr std::uint32_t kMaxStandardId = 0x7FF;

struct CanFrame {
  std::uint32_t id{0};                 // 11 位标准 ID
  std::uint8_t dlc{0};                 // 数据字节数 0..8
  std::array<std::uint8_t, kMaxDlc> data{};
};

class ICanBus {
 public:
  virtual ~ICanBus() = default;
  // 发送一帧。成功交给驱动返回 true（不代表别的节点收到）
  virtual bool send_frame(const CanFrame& f) = 0;
  // 非阻塞接收：有帧返回 true，没有立刻返回 false
  virtual bool receive_frame(CanFrame& out) = 0;
};

}  // namespace teledrive::can
