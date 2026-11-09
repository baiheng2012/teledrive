#include "teledrive/sim/lidar.hpp"

#include <algorithm>
#include <cmath>

namespace teledrive::sim {

namespace {

// 射线与圆求交。射线：origin + t·dir（t ≥ 0，dir 是单位向量）。
// 返回最近交点的 t；没有交点返回 -1。
//
// 推导：令 m = origin - center，解 |m + t·dir|² = r²
//   t² + 2(m·dir)t + (m·m - r²) = 0
//   b = m·dir，c = m·m - r²，判别式 = b² - c
double ray_circle(const Vec2& origin, const Vec2& dir, const Circle& circle) {
  const Vec2 m = origin - circle.center;
  const double c = m.dot(m) - circle.radius * circle.radius;
  if (c <= 0.0) {
    return 0.0;           // 起点已经在圆内：认为"距离为 0"（已经撞上）
  }
  const double b = m.dot(dir);
  if (b > 0.0) {
    return -1.0;          // 起点在圆外，且射线背离圆心：不可能相交
  }
  const double disc = b * b - c;
  if (disc < 0.0) {
    return -1.0;          // 射线擦不到圆
  }
  return -b - std::sqrt(disc);  // 取较近的那个交点（进入圆的点）
}

}  // namespace

Lidar::Lidar(LidarConfig config) : config_(config) {
  // 规范化配置：防止外部传入离谱的值导致数组越界或除零
  config_.beams = std::clamp<std::size_t>(config_.beams, 1, kMaxBeams);
  if (!(config_.max_range > 0.0)) config_.max_range = 1.0;   // 同时拦截 NaN
  if (!(config_.fov > 0.0)) config_.fov = kPi;
}

LidarScan Lidar::scan(const World& world, const Pose& pose) const {
  LidarScan s;
  s.count = config_.beams;
  s.max_range = config_.max_range;
  s.angle_min = -config_.fov / 2.0;
  // 只有 1 根射线时没有"间隔"可言，避免除以 0
  s.angle_increment =
      s.count > 1 ? config_.fov / static_cast<double>(s.count - 1) : 0.0;

  for (std::size_t i = 0; i < s.count; ++i) {
    // 射线的世界角度 = 车头朝向 + 射线相对角度
    const double a = pose.heading + s.angle_at(i);
    const Vec2 dir{std::cos(a), std::sin(a)};

    double range = s.max_range;   // 默认：没命中
    for (const auto& obstacle : world.obstacles) {
      const double t = ray_circle(pose.position, dir, obstacle);
      if (t >= 0.0 && t < range) {
        range = t;
      }
    }
    s.ranges[i] = range;
  }
  return s;
}

}  // namespace teledrive::sim
