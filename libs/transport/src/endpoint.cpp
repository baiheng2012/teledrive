#include "teledrive/transport/endpoint.hpp"

#include <arpa/inet.h>  // inet_pton、ntohl（POSIX/Linux）

namespace teledrive::transport {

std::optional<Endpoint> make_endpoint(const std::string& dotted_ip, std::uint16_t port) {
  in_addr addr{};
  // inet_pton：把文本地址转成二进制。成功返回 1，格式不合法返回 0。
  if (inet_pton(AF_INET, dotted_ip.c_str(), &addr) != 1) {
    return std::nullopt;
  }
  // inet_pton 给出的是网络字节序（大端），转成我们约定的主机字节序
  return Endpoint{ntohl(addr.s_addr), port};
}

std::string to_string(const Endpoint& e) {
  // 把 32 位整数拆成 4 段，每段 8 位
  return std::to_string((e.ip >> 24) & 0xFF) + "." + std::to_string((e.ip >> 16) & 0xFF) +
         "." + std::to_string((e.ip >> 8) & 0xFF) + "." + std::to_string(e.ip & 0xFF) + ":" +
         std::to_string(e.port);
}

}  // namespace teledrive::transport
