#include "teledrive/control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

namespace teledrive::control {

namespace {

// 点 p 在线段 a→b 上的投影参数 t（未截断）：
//   t = 0 在 a 点，t = 1 在 b 点，t 在 (0,1) 之间表示投影落在线段内部。
// 公式：t = (p-a)·(b-a) / |b-a|²
double project_t(const Vec2& a, const Vec2& b, const Vec2& p) {
  const Vec2 d = b - a;
  const double len2 = d.dot(d);
  if (len2 < 1e-12) {
    return 0.0;  // 退化线段（两个路点重合）：当成一个点
  }
  return (p - a).dot(d) / len2;
}

Vec2 point_on(const Vec2& a, const Vec2& b, double t) { return a + (b - a) * t; }

// 点 p 到线段 a→b 的最短距离
double distance_to_segment(const Vec2& a, const Vec2& b, const Vec2& p) {
  const double t = std::clamp(project_t(a, b, p), 0.0, 1.0);
  return (p - point_on(a, b, t)).norm();
}

}  // namespace

void PurePursuit::set_path(std::vector<Vec2> path) {
  path_ = std::move(path);
  seg_ = 0;
  finished_ = false;
}

void PurePursuit::restart(const sim::Pose& pose) {
  finished_ = false;
  seg_ = 0;
  if (path_.size() < 2) {
    return;
  }
  // 全局搜索离车最近的路径段。严格小于号：距离相同时选靠前的段。
  double best = distance_to_segment(path_[0], path_[1], pose.position);
  for (std::size_t i = 1; i + 1 < path_.size(); ++i) {
    const double d = distance_to_segment(path_[i], path_[i + 1], pose.position);
    if (d < best) {
      best = d;
      seg_ = i;
    }
  }
}

PursuitOutput PurePursuit::update(const sim::Pose& pose) {
  PursuitOutput out;
  if (path_.size() < 2 || finished_) {
    out.finished = true;   // 没有路径或已完成：输出零速度
    return out;
  }

  const Vec2 pos = pose.position;
  const std::size_t last = path_.size() - 2;   // 最后一段的下标

  // ---- 1) 推进路径进度（只增不减）----
  // 只要"下一段比当前段离车更近"就前进。
  // 注意不能用"投影是否越过当前段终点"来判断：车在转角处会切弯，
  // 可能还没到转角就已经在转向下一段，那时当前段的投影还没越过终点，
  // 追逐点就会落到车的后面，车会莫名其妙地原地转向。
  while (seg_ < last && distance_to_segment(path_[seg_ + 1], path_[seg_ + 2], pos) <=
                            distance_to_segment(path_[seg_], path_[seg_ + 1], pos)) {
    ++seg_;
  }

  const Vec2 a = path_[seg_];
  const Vec2 b = path_[seg_ + 1];
  const double t = std::clamp(project_t(a, b, pos), 0.0, 1.0);
  const Vec2 proj = point_on(a, b, t);
  out.cross_track = (pos - proj).norm();

  // ---- 2) 终点判定：必须已经在最后一段 ----
  const Vec2 goal = path_.back();
  const double goal_dist = (goal - pos).norm();
  if (seg_ == last && goal_dist <= config_.goal_tolerance) {
    finished_ = true;
    out.finished = true;
    return out;
  }

  // ---- 3) 找追逐点：从投影点出发，沿路径再走 Ld 的弧长 ----
  // 用"沿路径的弧长"而不是"以车为圆心的圆与路径求交"：
  // 后者在车离路径较远时可能无交点，需要额外的兜底；前者永远有定义。
  Vec2 target = goal;           // 路径剩余长度不足 Ld 时，追逐点就是终点
  Vec2 cur = proj;
  double remaining = config_.lookahead;
  for (std::size_t i = seg_; i <= last; ++i) {
    const Vec2 end = path_[i + 1];
    const double dist = (end - cur).norm();
    if (dist >= remaining && dist > 1e-9) {
      target = cur + (end - cur) * (remaining / dist);
      break;
    }
    remaining -= dist;
    cur = end;
  }
  out.lookahead = target;

  // ---- 4) 转到车体坐标系：x_b 向前，y_b 向左 ----
  const Vec2 d = target - pos;
  const double c = std::cos(pose.heading);
  const double s = std::sin(pose.heading);
  const double xb = c * d.x + s * d.y;
  const double yb = -s * d.x + c * d.y;
  const double l2 = xb * xb + yb * yb;

  if (xb <= 0.0 || l2 < 1e-9) {
    // 追逐点在车身后（或与车重合）：圆弧公式在这里没有意义，原地转向对准。
    out.linear = 0.0;
    out.angular = (yb >= 0.0 ? 1.0 : -1.0) * config_.turn_in_place_rate;
    return out;
  }

  // ---- 5) 曲率与速度 ----
  // 车沿圆弧到达追逐点所需的曲率：κ = 2·y_b / L²（L 是车到追逐点的距离）
  double v = std::min(config_.cruise_speed, config_.goal_slowdown_gain * goal_dist);
  const double curvature = 2.0 * yb / l2;
  double w = v * curvature;

  // 角速度超限：线速度同比例缩小，保持同样的圆弧形状（只是走得慢一点）
  if (std::abs(w) > config_.max_angular) {
    v *= config_.max_angular / std::abs(w);
    w = std::copysign(config_.max_angular, w);
  }
  out.linear = v;
  out.angular = w;
  return out;
}

}  // namespace teledrive::control
