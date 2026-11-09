// 离线仿真：不用网络，直接跑"航向保持 + 车辆模型"，把轨迹输出成 CSV。
// 用法：sim_trace [with_hold|no_hold] > trace.csv
// 输出列：t,x,y,heading,linear,angular_cmd
#include <cstdio>
#include <cstring>

#include "teledrive/control/heading_hold.hpp"
#include "teledrive/sim/diff_drive.hpp"

int main(int argc, char** argv) {
  const bool with_hold = !(argc > 1 && std::strcmp(argv[1], "no_hold") == 0);
  constexpr double kDt = 0.02;
  constexpr double kDisturbance = 0.05;  // 恒定偏航扰动 rad/s，模拟单侧轮打滑

  teledrive::sim::DiffDriveModel car;
  teledrive::control::HeadingHold hold;

  std::printf("t,x,y,heading,linear,angular_cmd\n");
  for (int i = 0; i < 1500; ++i) {  // 30 秒
    const auto& s = car.state();
    double w = 0.0;
    if (with_hold) w = hold.update(0.0, s.pose.heading, s.linear_velocity, kDt);

    teledrive::protocol::Command c;
    c.linear = 1.0f;
    c.angular = static_cast<float>(w + kDisturbance);
    car.step(c, kDt);

    const auto& n = car.state();
    std::printf("%.3f,%.5f,%.5f,%.5f,%.5f,%.5f\n", i * kDt, n.pose.position.x,
                n.pose.position.y, n.pose.heading, n.linear_velocity, c.angular);
  }
  return 0;
}
