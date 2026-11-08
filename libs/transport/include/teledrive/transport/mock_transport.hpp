#pragma once
// 纯内存的传输实现，只用于测试。
// 价值：
//   1) 不依赖真实网络，测试快速且确定（不会因为端口占用、防火墙而失败）
//   2) 可以"故意制造故障"：丢包、注入损坏数据，用来验证安全机制
// 用法：两个 MockTransport 互相 connect，a.send() 的数据会直接进入 b 的收件箱。

#include <deque>

#include "teledrive/transport/transport.hpp"

namespace teledrive::transport {

class MockTransport final : public ITransport {
 public:
  explicit MockTransport(Endpoint self) : self_(self) {}

  // 指定对端：之后 send() 发出的数据进入 peer 的收件箱
  // （原始指针不拥有对象，要求 peer 比本对象活得久；测试里都是栈对象，满足）
  void connect(MockTransport* peer) { peer_ = peer; }

  // ---- 故障注入 ----
  // 接下来的 n 次 send 假装发出去了（返回 true），实际被"网络"吞掉。模拟 UDP 丢包。
  void drop_next_sends(int n) { drop_remaining_ = n; }

  // 直接往自己的收件箱塞一个数据报，模拟收到了任意数据（包括损坏的、恶意的包）
  void inject(const Endpoint& from, const std::uint8_t* data, std::size_t len);

  // 统计：调用过多少次 send（含被丢弃的），测试里用来断言
  std::size_t sent_count() const { return sent_count_; }

  bool send(const Endpoint& to, const std::uint8_t* data, std::size_t len) override;
  bool receive(Datagram& out) override;

 private:
  Endpoint self_;
  MockTransport* peer_{nullptr};
  std::deque<Datagram> inbox_;   // 先进先出：和真实网络的到达顺序一致
  int drop_remaining_{0};
  std::size_t sent_count_{0};
};

}  // namespace teledrive::transport
