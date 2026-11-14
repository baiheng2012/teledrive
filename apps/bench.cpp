// 性能基准：协议编解码、控制周期耗时、UDP 回环往返。
// 用法（必须 Release 构建）：./build-release/apps/bench
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "teledrive/common/clock.hpp"
#include "teledrive/node/vehicle_node.hpp"
#include "teledrive/protocol/messages.hpp"
#include "teledrive/transport/mock_transport.hpp"
#include "teledrive/transport/udp_transport.hpp"

namespace {

using Clock = std::chrono::steady_clock;
namespace proto = teledrive::protocol;
namespace tr = teledrive::transport;

double us_between(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::micro>(b - a).count();
}

// 对一组耗时样本（微秒）输出 p50 / p99 / max
void report(const char* name, std::vector<double>& s) {
  std::sort(s.begin(), s.end());
  const auto at = [&](double q) { return s[static_cast<std::size_t>(q * (s.size() - 1))]; };
  std::printf("%-34s p50 %8.2f us | p99 %8.2f us | max %9.2f us  (n=%zu)\n", name, at(0.50),
              at(0.99), s.back(), s.size());
}

// 1) 协议：encode + decode 一条 Command 的吞吐
void bench_protocol() {
  constexpr int kN = 2000000;
  volatile float sink = 0;   // volatile：防止编译器发现结果没人用而把整个循环优化掉
  proto::Command c;
  c.linear = 0.5f;
  c.angular = 0.1f;
  c.flags = proto::kFlagArm;

  const auto t0 = Clock::now();
  for (int i = 0; i < kN; ++i) {
    c.linear = static_cast<float>(i & 0xFF) * 0.001f;
    const auto enc = proto::encode(static_cast<std::uint16_t>(i), c);
    const auto res = proto::decode(enc.data.data(), enc.size);
    sink = sink + std::get<proto::Command>(res.frame.payload).linear;
  }
  const double total_us = us_between(t0, Clock::now());
  std::printf("%-34s %.0f ns per round trip, %.2f M/s\n", "protocol encode+decode (Command)",
              total_us * 1000.0 / kN, kN / total_us);
}

// 2) 控制周期：node.tick() 的耗时。包含收包、解码、安全层、避障（可选）、车辆模型、遥测。
void bench_tick(bool assist, const char* name) {
  constexpr int kTicks = 20000;
  teledrive::FakeClock clock;
  const tr::Endpoint op_ep{0x7F000001u, 9000}, veh_ep{0x7F000001u, 9001};
  tr::MockTransport op_t(op_ep), veh_t(veh_ep);
  op_t.connect(&veh_t);
  veh_t.connect(&op_t);

  teledrive::node::VehicleNodeConfig cfg;
  cfg.world.obstacles.push_back(teledrive::sim::Circle{teledrive::Vec2{5.0, 0.0}, 0.5});
  cfg.obstacle_assist_enabled = assist;
  teledrive::node::VehicleNode node(veh_t, clock, cfg);

  std::vector<double> samples;
  samples.reserve(kTicks);
  tr::Datagram sink;
  for (int i = 0; i < kTicks; ++i) {
    proto::Command c;
    c.flags = proto::kFlagArm;
    c.linear = i < 2 ? 0.0f : 1.0f;
    const auto enc = proto::encode(static_cast<std::uint16_t>(i), c);
    veh_t.inject(op_ep, enc.data.data(), enc.size);   // 每个周期收到一条指令

    const auto t0 = Clock::now();
    node.tick();                                      // 只计时这一行
    samples.push_back(us_between(t0, Clock::now()));

    while (op_t.receive(sink)) {}                     // 清空遥测，避免收件箱无限增长
    clock.advance(std::chrono::milliseconds(20));
  }
  report(name, samples);
}

// 3) 真实 UDP 回环：A 发心跳 -> B 收到后回发 -> A 收到。忙等轮询，测的是系统调用加内核回环的开销。
void bench_udp_rtt() {
  constexpr int kN = 20000;
  auto a = tr::UdpTransport::open(0);
  auto b = tr::UdpTransport::open(0);
  if (!a || !b) {
    std::printf("udp bench skipped: cannot open sockets\n");
    return;
  }
  const auto to_a = *tr::make_endpoint("127.0.0.1", a->local_port());
  const auto to_b = *tr::make_endpoint("127.0.0.1", b->local_port());
  const auto enc = proto::encode(1, proto::Heartbeat{1});

  // 在 deadline 内忙等收一个包；超时返回 false（用来识别丢包，避免死循环）
  auto wait_for = [](tr::UdpTransport& t, tr::Datagram& d) {
    const auto deadline = Clock::now() + std::chrono::milliseconds(100);
    while (Clock::now() < deadline) {
      if (t.receive(d)) return true;
    }
    return false;
  };

  std::vector<double> samples;
  samples.reserve(kN);
  int lost = 0;
  tr::Datagram d;
  for (int i = 0; i < kN; ++i) {
    const auto t0 = Clock::now();
    a->send(to_b, enc.data.data(), enc.size);
    if (!wait_for(*b, d)) { ++lost; continue; }
    b->send(to_a, enc.data.data(), enc.size);
    if (!wait_for(*a, d)) { ++lost; continue; }
    samples.push_back(us_between(t0, Clock::now()));
  }
  report("UDP loopback round trip", samples);
  if (lost > 0) std::printf("  (%d lost)\n", lost);
}

}  // namespace

int main() {
#ifndef NDEBUG
  std::printf("WARNING: not a Release build, numbers are not meaningful\n");
#endif
  std::printf("Control budget per cycle: 20000 us (50 Hz)\n\n");
  bench_protocol();
  bench_tick(false, "VehicleNode::tick (assist off)");
  bench_tick(true, "VehicleNode::tick (lidar assist on)");
  bench_udp_rtt();
  return 0;
}
