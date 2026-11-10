// 车辆端程序：监听 UDP 端口，接收操作端指令，运行仿真车，回传遥测。
// 用法：vehicle_sim [--port 9001]
// 这个文件只做"接线"：创建真实的 UDP 和时钟，交给 VehicleNode。逻辑都在 libs/node。

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "teledrive/common/clock.hpp"
#include "teledrive/common/periodic.hpp"
#include "teledrive/node/names.hpp"
#include "teledrive/node/vehicle_node.hpp"
#include "teledrive/transport/udp_transport.hpp"

namespace {

// 信号处理函数里只能安全地做极少的事。设置一个 sig_atomic_t 标志，主循环检查它，
// 这样 Ctrl-C 时程序能正常退出，析构函数会运行（socket 被正确关闭）。
volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

bool parse_port(const char* s, std::uint16_t& out) {
  char* end = nullptr;
  const long v = std::strtol(s, &end, 10);
  if (end == s || *end != '\0' || v < 1 || v > 65535) return false;  // 必须整串都是数字
  out = static_cast<std::uint16_t>(v);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::uint16_t port = 9001;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      if (!parse_port(argv[++i], port)) {
        std::fprintf(stderr, "invalid port: %s\n", argv[i]);
        return 2;
      }
    } else {
      std::fprintf(stderr, "usage: %s [--port N]\n", argv[0]);
      return 2;
    }
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::string err;
  auto udp = teledrive::transport::UdpTransport::open(port, &err);
  if (!udp) {
    std::fprintf(stderr, "cannot open UDP port %u: %s\n", static_cast<unsigned>(port), err.c_str());
    return 1;
  }

  teledrive::SteadyClock clock;
//  teledrive::node::VehicleNode node(*udp, clock);
  

  teledrive::node::VehicleNodeConfig config;
  // 在正前方 5m 处放一个半径 0.5m 的障碍物，用来演示避障限速
  config.world.obstacles.push_back(teledrive::sim::Circle{teledrive::Vec2{5.0, 0.0}, 0.5});
  config.path = {teledrive::Vec2{0, 0}, teledrive::Vec2{3, 0}, teledrive::Vec2{3, 3},
                 teledrive::Vec2{0, 3}, teledrive::Vec2{0, 0.6}};
  teledrive::node::VehicleNode node(*udp, clock, config);
  
  std::printf("vehicle_sim listening on UDP port %u  (Ctrl-C to quit)\n", static_cast<unsigned>(port));


  teledrive::Periodic loop(std::chrono::milliseconds(20));  // 50Hz
  unsigned tick = 0;
  while (!g_stop) {
    node.tick();

    // 每 0.5 秒刷新一次状态行。\r 回到行首覆盖旧内容，\033[K 清除行尾残留。
    if (++tick % 25 == 0) {
      const auto& s = node.model().state();
      const auto& st = node.stats();
      
      //std::printf("\r[%-7s] x=%6.2f y=%6.2f v=%5.2f bat=%5.1f%% | rx ok=%llu bad=%llu stale=%llu | fault=%s\033[K",
      //            teledrive::node::name(node.safety().state()), s.pose.position.x,
      //            s.pose.position.y, s.linear_velocity, s.battery_pct,
      //            static_cast<unsigned long long>(st.rx_ok), static_cast<unsigned long long>(st.rx_bad),
      //            static_cast<unsigned long long>(st.rx_stale),
      //            teledrive::node::name(node.safety().fault_reason()));
      
      std::printf("\r[%-7s] x=%6.2f y=%6.2f v=%5.2f bat=%4.1f%% rx=%llu bad=%llu %s %s\033[K",
              teledrive::node::name(node.safety().state()), s.pose.position.x,
              s.pose.position.y, s.linear_velocity, s.battery_pct,
              static_cast<unsigned long long>(st.rx_ok),
              static_cast<unsigned long long>(st.rx_bad),
              teledrive::node::name(node.safety().fault_reason()),
              node.obstacle_limited() ? "OBSTACLE-LIMIT"
                      : (node.auto_active() ? (node.path_finished() ? "AUTO-DONE" : "AUTO") : ""));



      std::fflush(stdout);
    }
    loop.wait();
  }
  std::printf("\nbye\n");
  return 0;
}
