// 操作端程序：读键盘 → 发指令 → 显示遥测。
// 用法：operator_cli [--vehicle 127.0.0.1:9001]

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>

#include "teledrive/common/clock.hpp"
#include "teledrive/common/periodic.hpp"
#include "teledrive/node/manual_input.hpp"
#include "teledrive/node/names.hpp"
#include "teledrive/node/operator_session.hpp"
#include "teledrive/transport/udp_transport.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

// RAII 管理终端模式：构造时切到"原始模式"（按键立刻可读、不回显），
// 析构时恢复。就算程序异常退出，只要栈正常展开，终端也不会被留在坏状态。
class RawTerminal {
 public:
  RawTerminal() {
    if (tcgetattr(STDIN_FILENO, &orig_) == 0) {
      termios raw = orig_;
      raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);  // 关闭行缓冲和回显
      raw.c_cc[VMIN] = 0;   // read 没数据时立刻返回 0，不阻塞
      raw.c_cc[VTIME] = 0;
      ok_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
    }
  }
  ~RawTerminal() {
    if (ok_) tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
  }
  RawTerminal(const RawTerminal&) = delete;
  RawTerminal& operator=(const RawTerminal&) = delete;
  bool ok() const { return ok_; }

 private:
  termios orig_{};
  bool ok_{false};
};

bool parse_port(const char* s, std::uint16_t& out) {
  char* end = nullptr;
  const long v = std::strtol(s, &end, 10);
  if (end == s || *end != '\0' || v < 1 || v > 65535) return false;
  out = static_cast<std::uint16_t>(v);
  return true;
}

// "127.0.0.1:9001" → Endpoint
std::optional<teledrive::transport::Endpoint> parse_endpoint(const std::string& s) {
  const auto colon = s.rfind(':');
  if (colon == std::string::npos) return std::nullopt;
  std::uint16_t port = 0;
  if (!parse_port(s.substr(colon + 1).c_str(), port)) return std::nullopt;
  return teledrive::transport::make_endpoint(s.substr(0, colon), port);
}

}  // namespace

int main(int argc, char** argv) {
  std::string vehicle_arg = "127.0.0.1:9001";
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--vehicle") == 0 && i + 1 < argc) {
      vehicle_arg = argv[++i];
    } else {
      std::fprintf(stderr, "usage: %s [--vehicle IP:PORT]\n", argv[0]);
      return 2;
    }
  }
  const auto vehicle = parse_endpoint(vehicle_arg);
  if (!vehicle) {
    std::fprintf(stderr, "invalid --vehicle address: %s\n", vehicle_arg.c_str());
    return 2;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::string err;
  auto udp = teledrive::transport::UdpTransport::open(0, &err);  // 本地端口让系统分配
  if (!udp) {
    std::fprintf(stderr, "cannot open UDP socket: %s\n", err.c_str());
    return 1;
  }

  teledrive::SteadyClock clock;
  teledrive::node::OperatorSession session(*udp, clock, *vehicle);
  teledrive::node::ManualInput input;

  std::printf("TeleDrive operator -> %s\n", teledrive::transport::to_string(*vehicle).c_str());
 // std::printf("keys: e=arm/disarm  w/s=speed  a/d=turn  space=zero  x=E-STOP  r=reset  q=quit\n\n");
  std::printf("keys: e=arm/disarm  m=auto  w/s=speed  a/d=turn  space=zero  x=E-STOP  r=reset  q=quit\n\n");
  {
    RawTerminal terminal;  // 作用域结束时自动恢复终端
    if (!terminal.ok()) {
      std::fprintf(stderr, "warning: stdin is not a terminal, keyboard control unavailable\n");
    }

    teledrive::Periodic loop(std::chrono::milliseconds(20));  // 50Hz
    unsigned tick = 0;
    while (!g_stop && !input.quit_requested()) {
      // 1) 读取这个周期内所有按下的键（read 非阻塞，没键就返回 0）
      char c;
      while (read(STDIN_FILENO, &c, 1) == 1) {
        input.handle_key(c);
      }

      // 2) 每个周期都发指令（即使没变化）：车辆端看门狗靠它"续命"
      session.send_command(input.next_command());
      if (tick % 10 == 0) session.send_heartbeat();  // 5Hz 心跳，用于测 RTT
      session.poll();

      // 3) 每 0.1 秒刷新显示
      if (tick % 5 == 0) {
        const auto t = session.latest_telemetry();
        const bool alive = session.link_alive(std::chrono::milliseconds(500));
        const double rtt_ms =
            session.rtt() ? std::chrono::duration<double, std::milli>(*session.rtt()).count() : -1.0;
        if (t) {
          std::printf("\r%s %-7s x=%6.2f y=%6.2f hdg=%5.2f v=%5.2f bat=%3u%% rtt=%5.1fms [%s]\033[K",
                      alive ? "LINK" : "LOST", teledrive::node::name(t->state), t->x, t->y,
                      t->heading, t->speed, static_cast<unsigned>(t->battery_pct), rtt_ms,
                      input.auto_mode() ? "AUTO" : (input.armed() ? "armed" : "disarmed"));
        } else {
          std::printf("\rwaiting for vehicle...\033[K");
        }
        std::fflush(stdout);
      }
      ++tick;
      loop.wait();
    }

    // 退出前发几个"解除 arm"的指令，让车辆回到 Idle，而不是等看门狗超时进入 Fault
    teledrive::protocol::Command disarm;
    for (int i = 0; i < 5; ++i) session.send_command(disarm);
  }
  std::printf("\nbye\n");
  return 0;
}
