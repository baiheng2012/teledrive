#include "teledrive/control/speed_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace teledrive::control {

double nearest_obstacle_ahead(const sim::LidarScan& scan, const SpeedLimiterConfig& cfg) {
  const double half_corridor = cfg.half_width + cfg.side_margin;
  double nearest = kInf;

  for (std::size_t i = 0; i < scan.count; ++i) {
    if (!scan.hit(i)) continue;                 // 没有回波的射线跳过

    // 把极坐标 (距离, 角度) 转成车体坐标系：x 向前，y 向左
    const double a = scan.angle_at(i);
    const double x = scan.ranges[i] * std::cos(a);
    const double y = scan.ranges[i] * std::sin(a);

    // 只关心"在前方、且横向偏移落在走廊内"的点。
    // 用前向距离 x 而不是射线长度：斜着的射线长度比实际前向距离长，
    // 用它会高估距离，让车刹得太晚。
    if (x > 0.0 && std::abs(y) <= half_corridor) {
      nearest = std::min(nearest, x);
    }
  }
  return nearest;
}

double allowed_forward_speed(double nearest, const SpeedLimiterConfig& cfg) {
  if (std::isinf(nearest)) return kInf;          // 前方没有障碍物：不限速
  const double room = nearest - cfg.stop_distance;   // 还能往前走多少
  if (room <= 0.0 || cfg.decel <= 0.0) return 0.0;   // 没空间（或配置无效）：必须停
  return std::sqrt(2.0 * cfg.decel * room);          // v² = 2·a·s
}

AssistResult limit_for_obstacles(const protocol::Command& in, const sim::LidarScan& scan,
                                 const SpeedLimiterConfig& cfg) {
  AssistResult r;
  r.command = in;                                   // 先原样拷贝：angular、flags 都保留
  r.nearest = nearest_obstacle_ahead(scan, cfg);
  r.allowed = allowed_forward_speed(r.nearest, cfg);

  // 只处理前进：允许速度比操作员想要的更小，才介入（只会压低，不会放大）
  if (in.linear > 0.0f && r.allowed < static_cast<double>(in.linear)) {
    r.command.linear = static_cast<float>(r.allowed);
    r.limited = true;
  }
  return r;
}

}  // namespace teledrive::control
