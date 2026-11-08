#include "teledrive/transport/udp_transport.hpp"

#include <arpa/inet.h>   // htons、htonl、ntohs、ntohl
#include <netinet/in.h>  // sockaddr_in
#include <sys/socket.h>  // socket、bind、sendto、recvfrom
#include <unistd.h>      // close

#include <cerrno>
#include <cstring>

namespace teledrive::transport {

std::unique_ptr<UdpTransport> UdpTransport::open(std::uint16_t local_port, std::string* error) {
  // 记录出错信息的小工具。必须在任何可能改变 errno 的调用（比如 close）之前使用。
  auto set_error = [error](const char* what) {
    if (error != nullptr) {
      *error = std::string(what) + ": " + std::strerror(errno);
    }
  };

  // 创建 UDP socket：
  //   AF_INET        : IPv4
  //   SOCK_DGRAM     : 数据报（UDP）
  //   SOCK_NONBLOCK  : 非阻塞，recvfrom 没数据时立刻返回而不是等待
  //   SOCK_CLOEXEC   : 如果程序 exec 其他程序，不把这个 fd 泄漏给子进程
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    set_error("socket");
    return nullptr;
  }

  // 绑定本地地址：监听本机所有网卡上的 local_port
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(local_port);  // 端口要转成网络字节序（大端）
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
    set_error("bind");  // 先记录错误，再 close（close 可能覆盖 errno）
    ::close(fd);        // 失败路径也要释放资源，否则 fd 泄漏
    return nullptr;
  }

  // 查询实际端口：local_port 为 0 时，系统会自动选一个，要问出来
  socklen_t len = sizeof(addr);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) < 0) {
    set_error("getsockname");
    ::close(fd);
    return nullptr;
  }

  // 构造函数是私有的，所以不能用 make_unique，这里直接 new
  return std::unique_ptr<UdpTransport>(new UdpTransport(fd, ntohs(addr.sin_port)));
}

void UdpTransport::close_fd() noexcept {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

UdpTransport::~UdpTransport() { close_fd(); }

// 移动构造：接管对方的 fd，并把对方置空，保证同一个 fd 只有一个主人
UdpTransport::UdpTransport(UdpTransport&& other) noexcept
    : fd_(other.fd_), local_port_(other.local_port_) {
  other.fd_ = -1;
}

UdpTransport& UdpTransport::operator=(UdpTransport&& other) noexcept {
  if (this != &other) {  // 防止自赋值：自己 close 了自己就坏了
    close_fd();          // 先释放自己原来持有的资源
    fd_ = other.fd_;
    local_port_ = other.local_port_;
    other.fd_ = -1;
  }
  return *this;
}

bool UdpTransport::send(const Endpoint& to, const std::uint8_t* data, std::size_t len) {
  // 参数检查：fd 无效（被移走）、空数据、超长都拒绝
  if (fd_ < 0 || data == nullptr || len == 0 || len > kMaxDatagramSize) {
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(to.ip);  // 主机字节序 -> 网络字节序
  addr.sin_port = htons(to.port);

  // sendto 返回实际发送的字节数；UDP 要么整包发出，要么失败
  const ssize_t n = ::sendto(fd_, data, len, 0, reinterpret_cast<const sockaddr*>(&addr),
                             sizeof(addr));
  return n == static_cast<ssize_t>(len);
}

bool UdpTransport::receive(Datagram& out) {
  if (fd_ < 0) {
    return false;
  }

  // 循环的目的：遇到"被信号打断"或"超长包"时跳过它，继续尝试下一个
  for (;;) {
    sockaddr_in src{};
    socklen_t src_len = sizeof(src);

    // MSG_TRUNC：即使包比缓冲区大，返回值也是包的"真实长度"，
    // 这样我们能识别出被截断的超长包并丢弃，而不是把半截数据当成完整包交给上层。
    const ssize_t n = ::recvfrom(fd_, out.data.data(), out.data.size(), MSG_TRUNC,
                                 reinterpret_cast<sockaddr*>(&src), &src_len);
    if (n < 0) {
      if (errno == EINTR) {
        continue;  // 被信号打断，重试
      }
      // EAGAIN / EWOULDBLOCK：当前没有数据（非阻塞的正常情况）；其他错误也当作没数据
      return false;
    }
    if (static_cast<std::size_t>(n) > out.data.size()) {
      continue;  // 超长包：丢弃，看下一个
    }

    out.size = static_cast<std::size_t>(n);
    out.from = Endpoint{ntohl(src.sin_addr.s_addr), ntohs(src.sin_port)};
    return true;
  }
}

}  // namespace teledrive::transport
