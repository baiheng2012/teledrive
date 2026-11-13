// 车辆端程序：监听 UDP 端口，接收操作端指令，运行仿真车，回传遥测，并推送车载画面。
// 用法：vehicle_sim [--port 9001]
//   控制端口 = --port，视频端口 = --port + 1（独立 socket，视频拥塞不影响控制）。
// 这个文件只做"接线"，逻辑都在 libs/ 里。

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "teledrive/common/clock.hpp"
#include "teledrive/common/periodic.hpp"
#include "teledrive/node/names.hpp"
#include "teledrive/node/vehicle_node.hpp"
#include "teledrive/transport/udp_transport.hpp"

#ifdef TELEDRIVE_WITH_VISION
#include "teledrive/sim/lidar.hpp"
#include "teledrive/vision/video_sender.hpp"
#endif

namespace {

// 信号处理函数里只做一件事：置标志。主循环检查它，这样 Ctrl-C 后析构函数会运行。
volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

bool parse_port(const char* s, std::uint16_t& out) {
  char* end = nullptr;
  const long v = std::strtol(s, &end, 10);
  if (end == s || *end != '\0' || v < 1 || v > 65535) return false;
  out = static_cast<std::uint16_t>(v);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::uint16_t port = 9001;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      if (!parse_port(argv[++i], port) || port == 65535) {   // 视频端口要 +1
        std::fprintf(stderr, "invalid port: %s (must be 1..65534)\n", argv[i]);
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

  teledrive::node::VehicleNodeConfig config;
  // 正前方 5m 处放一个半径 0.5m 的障碍物，用来演示避障限速
  config.world.obstacles.push_back(teledrive::sim::Circle{teledrive::Vec2{5.0, 0.0}, 0.5});
  // 一条绕方形的演示路径（起点是车的初始位置）
  config.path = {teledrive::Vec2{0, 0}, teledrive::Vec2{3, 0}, teledrive::Vec2{3, 3},
                 teledrive::Vec2{0, 3}, teledrive::Vec2{0, 0.6}};

  teledrive::SteadyClock clock;
  teledrive::node::VehicleNode node(*udp, clock, config);

#ifdef TELEDRIVE_WITH_VISION
  // 先声明 socket，再声明 sender：销毁顺序相反，sender 先销毁，引用不会悬空
  std::unique_ptr<teledrive::transport::UdpTransport> video_udp =
      teledrive::transport::UdpTransport::open(static_cast<std::uint16_t>(port + 1), &err);
  std::unique_ptr<teledrive::vision::VideoSender> video;
  if (video_udp) {
    video = std::make_unique<teledrive::vision::VideoSender>(*video_udp, clock);
  } else {
    std::fprintf(stderr, "warning: video disabled, cannot open UDP port %u: %s\n",
                 static_cast<unsigned>(port + 1), err.c_str());
  }
  const teledrive::sim::Lidar lidar(config.lidar);
#endif

  std::printf("vehicle_sim: control UDP %u, video UDP %u  (Ctrl-C to quit)\n",
              static_cast<unsigned>(port), static_cast<unsigned>(port + 1));

  teledrive::Periodic loop(std::chrono::milliseconds(20));  // 50Hz
  unsigned tick = 0;
  while (!g_stop) {
    node.tick();

#ifdef TELEDRIVE_WITH_VISION
    if (video) {
      video->poll();
      if (video->due()) {
        const auto& s = node.model().state();
        const auto scan = lidar.scan(config.world, s.pose);

        teledrive::vision::SceneView view;
        view.pose = s.pose;
        view.world = &config.world;
        view.path = &config.path;
        view.scan = &scan;

        teledrive::vision::HudInfo hud;
        hud.state = teledrive::node::name(node.safety().state());
        hud.mode = node.auto_active() ? "AUTO" : "MANUAL";
        if (node.safety().fault_reason() != teledrive::safety::FaultReason::None) {
          hud.note = teledrive::node::name(node.safety().fault_reason());
        } else if (node.obstacle_limited()) {
          hud.note = "OBSTACLE-LIMIT";
        }
        hud.speed = s.linear_velocity;
        hud.battery_pct = s.battery_pct;
        video->send_frame(view, hud);   // 渲染 + JPEG 约 4ms，在 20ms 预算内
      }
    }
#endif

    // 每 0.5 秒刷新一次状态行（控制在约 75 个字符内，窄终端也不会换行）
    if (++tick % 25 == 0) {
      const auto& s = node.model().state();
      const auto& st = node.stats();
      const char* tail = "";
      if (node.safety().fault_reason() != teledrive::safety::FaultReason::None) {
        tail = teledrive::node::name(node.safety().fault_reason());
      } else if (node.obstacle_limited()) {
        tail = "OBSTACLE-LIMIT";
      } else if (node.auto_active()) {
        tail = node.path_finished() ? "AUTO-DONE" : "AUTO";
      }
      unsigned long long vid = 0;
#ifdef TELEDRIVE_WITH_VISION
      if (video) vid = video->stats().frames_sent;
#endif
      std::printf("\r[%-7s] x=%6.2f y=%6.2f v=%5.2f rx=%llu bad=%llu vid=%llu %s\033[K",
                  teledrive::node::name(node.safety().state()), s.pose.position.x,
                  s.pose.position.y, s.linear_velocity,
                  static_cast<unsigned long long>(st.rx_ok),
                  static_cast<unsigned long long>(st.rx_bad), vid, tail);
      std::fflush(stdout);
    }
    loop.wait();
  }

  std::printf("\n");
#ifdef TELEDRIVE_WITH_VISION
  if (video) {
    const auto& vs = video->stats();
    std::printf("video: %llu frames, %llu packets | render max %.2f ms, jpeg max %.2f ms\n",
                static_cast<unsigned long long>(vs.frames_sent),
                static_cast<unsigned long long>(vs.packets_sent), vs.render_ms_max,
                vs.encode_ms_max);
  }
#endif
  std::printf("bye\n");
  return 0;
}
