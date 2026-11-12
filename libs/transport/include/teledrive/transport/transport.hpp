#pragma once
// 传输层接口。上层（车辆端、操作端）只依赖这个接口，
// 不关心底下是真实 UDP、内存模拟，还是以后的 CAN。
// 这就是"依赖倒置"：高层模块依赖抽象，不依赖具体实现。

#include <array>
#include <cstddef>
#include <cstdint>

#include "teledrive/transport/endpoint.hpp"

namespace teledrive::transport {

// 单个数据报的最大字节数。控制帧最大 74 字节；视频分片约 1215 字节；1500 = 以太网 MTU，避免 IP 层分片。
inline constexpr std::size_t kMaxDatagramSize = 1500;

// 一个收到的数据报：内容 + 来源地址。
// 用固定大小数组而不是 vector：收包发生在控制循环里，避免堆分配。
struct Datagram {
  std::array<std::uint8_t, kMaxDatagramSize> data{};
  std::size_t size{0};   // 有效字节数
  Endpoint from{};       // 发送方地址
};

class ITransport {
 public:
  // 虚析构：通过基类指针删除派生类对象时，必须能正确调用派生类的析构函数，
  // 否则 UdpTransport 的 socket 不会被关闭（泄漏）。
  virtual ~ITransport() = default;

  // 发送一个数据报。成功交给系统发送返回 true。
  // 注意：UDP 的 true 只代表"交给网卡了"，不代表对方收到。
  virtual bool send(const Endpoint& to, const std::uint8_t* data, std::size_t len) = 0;

  // 非阻塞接收：有数据就填入 out 并返回 true；没有就立刻返回 false，绝不等待。
  virtual bool receive(Datagram& out) = 0;
};

}  // namespace teledrive::transport
