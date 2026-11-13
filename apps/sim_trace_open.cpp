// 开环对照场景：固定的指令序列驱动默认参数的车辆模型，输出 CSV 供 MATLAB 交叉验证。
// 每一行：时刻 t_i、本步施加的指令、步骤开始时（施加指令之前）的状态。
// 最后追加一行 t_N 的最终状态（指令列为 0，无意义）。
// 指令以 %.9g 输出：这足以让 float 无损往返，MATLAB 读到的就是 C++ 实际用的值。
#include <cstdio>

#include "teledrive/sim/diff_drive.hpp"

namespace {
struct Cmd { float v, w; };
// 分段常值指令（w 始终 >= 0，原因见文档）：
//   0~2s   直行 1.0 m/s
//   2~6s   1.0 m/s 并以 0.5 rad/s 左转
//   6~8s   0.5 m/s 并以 0.8 rad/s 左转
//   8~10s  停车
Cmd schedule(int i) {
  if (i < 100) return {1.0f, 0.0f};
  if (i < 300) return {1.0f, 0.5f};
  if (i < 400) return {0.5f, 0.8f};
  return {0.0f, 0.0f};
}
}  // namespace

int main() {
  constexpr double kDt = 0.02;
  constexpr int kSteps = 500;
  teledrive::sim::DiffDriveModel car;   // 默认参数，与 MATLAB 里的常量一一对应

  std::printf("t,v_cmd,w_cmd,x,y,heading,v,w\n");
  for (int i = 0; i < kSteps; ++i) {
    const Cmd c = schedule(i);
    const auto& s = car.state();
    std::printf("%.2f,%.9g,%.9g,%.9f,%.9f,%.9f,%.9f,%.9f\n", i * kDt, c.v, c.w,
                s.pose.position.x, s.pose.position.y, s.pose.heading, s.linear_velocity,
                s.angular_velocity);
    teledrive::protocol::Command cmd;
    cmd.linear = c.v;
    cmd.angular = c.w;
    car.step(cmd, kDt);
  }
  const auto& s = car.state();
  std::printf("%.2f,0,0,%.9f,%.9f,%.9f,%.9f,%.9f\n", kSteps * kDt, s.pose.position.x,
              s.pose.position.y, s.pose.heading, s.linear_velocity, s.angular_velocity);
  return 0;
}
