#pragma once
// 避障限速：根据雷达扫描结果，夹住操作员的前进速度指令。
//
// 纯函数、无状态：同样的输入永远得到同样的输出，测试起来很简单。
// 只限制"前进"：雷达在前面，对后方一无所知。也不限制转向，让操作员能转走脱困。

#include <limits>

#include "teledrive/protocol/messages.hpp"
#include "teledrive/sim/lidar.hpp"

namespace teledrive::control {

struct SpeedLimiterConfig {
  double half_width{0.3};      // 车辆半宽 (m)
  double side_margin{0.15};    // 走廊两侧额外余量 (m)
  double stop_distance{0.6};   // 希望停在距障碍物多远处 (m)
  double decel{1.0};           // 设计减速度 (m/s²)，应小于车辆真实最大减速度，留余量
};

inline constexpr double kInf = std::numeric_limits<double>::infinity();

struct AssistResult {
  protocol::Command command{};   // 处理后的指令
  bool limited{false};           // 本次是否真的把速度压低了
  double nearest{kInf};          // 走廊内最近障碍物的前向距离，没有则为 kInf
  double allowed{kInf};          // 当前允许的最高前进速度，没有限制则为 kInf
};

// 走廊内最近障碍物的"前向距离"（沿车头方向的投影）。没有则返回 kInf。
double nearest_obstacle_ahead(const sim::LidarScan& scan, const SpeedLimiterConfig& cfg);

// 由最近距离算允许速度：v = sqrt(2·a·(d - stop_distance))，距离不足则为 0。
double allowed_forward_speed(double nearest, const SpeedLimiterConfig& cfg);

// 对操作员指令应用限速。只改 linear，flags 与 angular 原样保留。
AssistResult limit_for_obstacles(const protocol::Command& in, const sim::LidarScan& scan,
                                 const SpeedLimiterConfig& cfg);

}  // namespace teledrive::control
