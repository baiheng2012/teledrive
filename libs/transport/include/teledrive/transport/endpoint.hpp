#pragma once
// 网络地址：IPv4 地址 + 端口。
// 为什么自己定义，而不是直接用系统的 sockaddr_in？
//   1) sockaddr_in 是 Linux 专有类型，会把系统头文件泄漏给所有使用者
//   2) 以后换成 CAN，"地址"就变成 CAN ID，上层代码不应该依赖 sockaddr
// 约定：ip 和 port 都存"主机字节序"，只在真正调用 socket 函数时才转成网络字节序。

#include <cstdint>
#include <optional>
#include <string>

namespace teledrive::transport {

struct Endpoint {
  std::uint32_t ip{0};    // 例如 127.0.0.1 存成 0x7F000001
  std::uint16_t port{0};
};

inline bool operator==(const Endpoint& a, const Endpoint& b) {
  return a.ip == b.ip && a.port == b.port;
}
inline bool operator!=(const Endpoint& a, const Endpoint& b) { return !(a == b); }

// 把 "127.0.0.1" 这样的点分字符串解析成 Endpoint。
// 字符串来自用户输入，可能是错的，所以返回 optional：解析失败返回 std::nullopt。
std::optional<Endpoint> make_endpoint(const std::string& dotted_ip, std::uint16_t port);

// 转成 "127.0.0.1:9000" 形式，用于日志和调试。
std::string to_string(const Endpoint& e);

}  // namespace teledrive::transport
