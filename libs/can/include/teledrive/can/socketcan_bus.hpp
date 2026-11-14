#pragma once
// Linux SocketCAN 实现：通过 PF_CAN 原始套接字收发经典 CAN 帧。
// 和 UdpTransport 一样用 RAII 管理文件描述符，禁止拷贝和移动，用工厂函数创建。
// 默认行为：同一个接口上，其他套接字发的帧本套接字收得到，自己发的收不到（CAN_RAW_RECV_OWN_MSGS 关闭）。

#include <memory>
#include <string>

#include "teledrive/can/can_frame.hpp"

namespace teledrive::can {

class SocketCanBus final : public ICanBus {
 public:
  // 打开并绑定接口（如 "vcan0"、"can0"）。失败返回 nullptr，原因写入 *error。
  static std::unique_ptr<SocketCanBus> open(const std::string& ifname, std::string* error = nullptr);

  ~SocketCanBus() override;
  SocketCanBus(const SocketCanBus&) = delete;
  SocketCanBus& operator=(const SocketCanBus&) = delete;

  bool send_frame(const CanFrame& f) override;
  bool receive_frame(CanFrame& out) override;

 private:
  explicit SocketCanBus(int fd) : fd_(fd) {}
  int fd_{-1};
};

}  // namespace teledrive::can
