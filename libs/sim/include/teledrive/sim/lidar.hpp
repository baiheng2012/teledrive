#pragma once
// 仿真世界与 2D 激光雷达。
//
// 坐标约定与 diff_drive.hpp 一致：朝向 0 = +x，逆时针为正。
// 雷达装在车辆中心，射线角度是"相对车头"的：0 = 正前方，正数 = 左前方。
//
// 为什么 LidarScan 用固定数组而不是 vector？
//   扫描发生在 50Hz 控制循环里，每个周期都做一次，避免每次堆分配。

#include <array>
#include <cstddef>
#include <vector>

#include "teledrive/common/vec2.hpp"
#include "teledrive/sim/diff_drive.hpp"  // Pose、kPi

namespace teledrive::sim {

// 圆形障碍物
struct Circle {
  Vec2 center{};
  double radius{0.5};
};

// 仿真世界：目前只有圆形障碍物
struct World {
  std::vector<Circle> obstacles;
};

inline constexpr std::size_t kMaxBeams = 181;

struct LidarConfig {
  std::size_t beams{61};      // 射线数量，会被限制在 [1, kMaxBeams]
  double fov{kPi};            // 视场角（弧度），默认前方 180°（±90°）
  double max_range{5.0};      // 最大探测距离 (m)；没命中就返回这个值
};

// 一次扫描结果
struct LidarScan {
  std::array<double, kMaxBeams> ranges{};  // 每根射线的距离；== max_range 表示"没有回波"
  std::size_t count{0};                    // 有效射线数
  double angle_min{0.0};                   // 第 0 根射线相对车头的角度
  double angle_increment{0.0};             // 相邻射线夹角
  double max_range{0.0};

  double angle_at(std::size_t i) const { return angle_min + static_cast<double>(i) * angle_increment; }
  bool hit(std::size_t i) const { return ranges[i] < max_range; }
};

class Lidar {
 public:
  explicit Lidar(LidarConfig config = {});

  // 在给定位姿处做一次扫描。无副作用，不分配堆内存。
  LidarScan scan(const World& world, const Pose& pose) const;

  const LidarConfig& config() const { return config_; }

 private:
  LidarConfig config_;
};

}  // namespace teledrive::sim
