// 离线仿真：Pure Pursuit 沿方形路径行驶，输出轨迹 CSV。
// 输出列：t,x,y,heading,lookahead_x,lookahead_y,cross_track
#include <cstdio>
#include <vector>

#include "teledrive/control/pure_pursuit.hpp"
#include "teledrive/sim/diff_drive.hpp"

int main() {
  using teledrive::Vec2;
  // 注意：scripts 里的路径必须与 tools/plot_pure_pursuit.py 里的 PATH 保持一致
  const std::vector<Vec2> path{Vec2{0, 0}, Vec2{4, 0}, Vec2{4, 3}, Vec2{0, 3}, Vec2{0, 0.6}};

  teledrive::control::PurePursuit pp;
  pp.set_path(path);
  teledrive::sim::DiffDriveModel car;
  pp.restart(car.state().pose);

  constexpr double kDt = 0.02;
  std::printf("t,x,y,heading,lookahead_x,lookahead_y,cross_track\n");
  for (int i = 0; i < 6000; ++i) {   // 最多 120 秒
    const auto& s = car.state();
    const auto o = pp.update(s.pose);
    std::printf("%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", i * kDt, s.pose.position.x,
                s.pose.position.y, s.pose.heading, o.lookahead.x, o.lookahead.y, o.cross_track);
    if (o.finished) break;
    teledrive::protocol::Command c;
    c.linear = static_cast<float>(o.linear);
    c.angular = static_cast<float>(o.angular);
    car.step(c, kDt);
  }
  return 0;
}
