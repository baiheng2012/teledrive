#include "teledrive/can/socketcan_bus.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace teledrive::can {

std::unique_ptr<SocketCanBus> SocketCanBus::open(const std::string& ifname, std::string* error) {
  auto set_error = [error](const char* what) {
    if (error != nullptr) *error = std::string(what) + ": " + std::strerror(errno);
  };

  // SOCK_RAW + CAN_RAW：原始 CAN 帧；NONBLOCK：receive 没数据时立刻返回
  const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (fd < 0) {
    set_error("socket(PF_CAN)");
    return nullptr;
  }

  if (ifname.empty() || ifname.size() >= IFNAMSIZ) {
    if (error != nullptr) *error = "invalid interface name";
    ::close(fd);
    return nullptr;
  }
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {   // 接口名 -> 接口编号（接口不存在会失败）
    set_error(("interface " + ifname).c_str());
    ::close(fd);
    return nullptr;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
    set_error("bind");
    ::close(fd);
    return nullptr;
  }
  return std::unique_ptr<SocketCanBus>(new SocketCanBus(fd));
}

SocketCanBus::~SocketCanBus() {
  if (fd_ >= 0) ::close(fd_);
}

bool SocketCanBus::send_frame(const CanFrame& f) {
  if (fd_ < 0 || f.dlc > kMaxDlc || f.id > kMaxStandardId) return false;
  can_frame cf{};
  cf.can_id = f.id;   // 不带 CAN_EFF_FLAG：标准帧
  cf.len = f.dlc;
  std::memcpy(cf.data, f.data.data(), f.dlc);
  const ssize_t n = ::write(fd_, &cf, sizeof(cf));
  return n == static_cast<ssize_t>(sizeof(cf));   // 发送队列满时会失败（ENOBUFS）
}

bool SocketCanBus::receive_frame(CanFrame& out) {
  if (fd_ < 0) return false;
  for (;;) {
    can_frame cf{};
    const ssize_t n = ::read(fd_, &cf, sizeof(cf));
    if (n < 0) {
      if (errno == EINTR) continue;
      return false;   // EAGAIN：当前没有帧
    }
    if (n != static_cast<ssize_t>(sizeof(cf))) continue;   // 不是经典帧大小，丢弃
    // 扩展帧、远程帧、错误帧不属于我们的协议，丢弃
    if (cf.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) continue;
    if (cf.len > kMaxDlc) continue;
    out.id = cf.can_id & CAN_SFF_MASK;
    out.dlc = cf.len;
    out.data = {};
    std::memcpy(out.data.data(), cf.data, cf.len);
    return true;
  }
}

}  // namespace teledrive::can
