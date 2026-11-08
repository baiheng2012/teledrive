#pragma once
// 基于 Linux UDP socket 的传输实现。
// 用 RAII 管理文件描述符：构造时获得，析构时 close。
// 资源独占 => 禁止拷贝（否则两个对象会 close 同一个 fd），允许移动（转移所有权）。

#include <memory>
#include <string>

#include "teledrive/transport/transport.hpp"

namespace teledrive::transport {

class UdpTransport final : public ITransport {
 public:
  // 工厂函数：创建 socket 并绑定到本地端口。
  //   local_port = 0 表示让系统自动分配一个空闲端口（测试时很有用）
  // 为什么不用构造函数？构造函数不能返回"失败"，而我们不使用异常。
  // 失败时返回 nullptr，并把原因写到 *error（如果传了的话）。
  static std::unique_ptr<UdpTransport> open(std::uint16_t local_port,
                                            std::string* error = nullptr);

  ~UdpTransport() override;

  // Rule of Five：因为我们自己管理资源（fd_），拷贝/移动必须显式处理
  UdpTransport(const UdpTransport&) = delete;
  UdpTransport& operator=(const UdpTransport&) = delete;
  UdpTransport(UdpTransport&& other) noexcept;
  UdpTransport& operator=(UdpTransport&& other) noexcept;

  bool send(const Endpoint& to, const std::uint8_t* data, std::size_t len) override;
  bool receive(Datagram& out) override;

  // 实际绑定的端口（传 0 时，用它查出系统分配了哪个）
  std::uint16_t local_port() const { return local_port_; }

 private:
  // 私有构造：只能通过 open() 创建，保证对象一旦存在，fd 一定有效
  UdpTransport(int fd, std::uint16_t port) : fd_(fd), local_port_(port) {}
  void close_fd() noexcept;

  int fd_{-1};                 // -1 表示"没有持有任何资源"
  std::uint16_t local_port_{0};
};

}  // namespace teledrive::transport
