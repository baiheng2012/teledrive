#pragma once
// Pure Pursuit 路径跟踪。
//
// 输入：一条折线路径（路点序列）+ 车当前位姿
// 输出：线速度和角速度指令
//
// 与前面的模块一样是"纯计算"：不读时钟、不依赖网络，dt 之类都不需要，
// 因为它只做几何计算，每次调用只取决于路径、内部的"路径进度"和当前位姿。
//
// 内部状态只有两个：seg_（当前在第几段路径，只增不减）和 finished_（是否已到达终点）。

#include <cstddef>
#include <vector>

#include "teledrive/common/vec2.hpp"
#include "teledrive/sim/diff_drive.hpp"  // Pose

namespace teledrive::control {

struct PurePursuitConfig {
  double lookahead{0.8};            // 前视距离 Ld (m)：大 = 平滑但切弯，小 = 贴路径但易振荡
  double cruise_speed{0.6};         // 巡航线速度 (m/s)
  double max_angular{1.5};          // 角速度上限 (rad/s)，应不大于安全层的角速度限幅
  double turn_in_place_rate{0.8};   // 追逐点在车后方时，原地转向的角速度 (rad/s)
  double goal_tolerance{0.15};      // 离终点多近算到达 (m)
  double goal_slowdown_gain{1.0};   // 终点附近：v <= gain * 离终点距离
};

struct PursuitOutput {
  double linear{0.0};
  double angular{0.0};
  bool finished{false};     // 已到达终点（或没有有效路径），此时速度为零
  Vec2 lookahead{};         // 本次使用的追逐点（调试和画图用）
  double cross_track{0.0};  // 车到当前路径段的垂直距离（不带符号），评估跟踪精度用
};

class PurePursuit {
 public:
  explicit PurePursuit(PurePursuitConfig config = {}) : config_(config) {}

  // 设置新路径并复位进度。少于 2 个路点视为"没有路径"。
  void set_path(std::vector<Vec2> path);

  // 从当前位置重新开始：选离车最近的路径段作为起点，并清除"已完成"。
  // 切换到自动模式时调用，这样操作员手动开到半路再切自动，车会接着最近的路段走，
  // 而不是跑回路径起点。
  void restart(const sim::Pose& pose);

  // 计算一步。
  PursuitOutput update(const sim::Pose& pose);

  bool finished() const { return finished_; }
  std::size_t segment() const { return seg_; }
  bool has_path() const { return path_.size() >= 2; }

 private:
  PurePursuitConfig config_;
  std::vector<Vec2> path_;
  std::size_t seg_{0};
  bool finished_{false};
};

}  // namespace teledrive::control
