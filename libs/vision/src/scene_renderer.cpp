#include "teledrive/vision/scene_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <opencv2/imgproc.hpp>

namespace teledrive::vision {

namespace {

constexpr int kFar = 100000;      // 画面外的"很远"：坐标夹在这个范围内，避免整数溢出
constexpr int kHudHeight = 64;

cv::Scalar sc(const cv::Vec3b& c) { return cv::Scalar(c[0], c[1], c[2]); }

// double → int 像素：非有限值放到远处；有限值夹在 ±kFar 内再取整。
// 直接强转一个 1e300 的 double 是未定义行为，所以必须先夹住。
int clamp_px(double v) {
  if (!std::isfinite(v)) return -kFar;
  return static_cast<int>(std::lround(std::clamp(v, -1.0 * kFar, 1.0 * kFar)));
}

void put_text(cv::Mat& img, const std::string& text, int y, const cv::Vec3b& color) {
  cv::putText(img, text, cv::Point(8, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, sc(color), 1,
              cv::LINE_AA);
}

void draw_hud(cv::Mat& img, const HudInfo& hud) {
  cv::rectangle(img, cv::Rect(0, 0, img.cols, kHudHeight), sc(colors::kHudBar), cv::FILLED);
  char tmp[64];

  std::snprintf(tmp, sizeof(tmp), " | ");
  put_text(img, hud.state + tmp + hud.mode, 16,
           hud.state == "FAULT" ? colors::kTextFault : colors::kText);

  std::snprintf(tmp, sizeof(tmp), "v=%5.2f m/s   bat=%3.0f%%", hud.speed, hud.battery_pct);
  put_text(img, tmp, 34, colors::kText);

  std::string third = hud.note;
  if (hud.latency_ms >= 0.0) {
    std::snprintf(tmp, sizeof(tmp), "  lat=%.0fms", hud.latency_ms);
    third += tmp;
  }
  if (hud.fps >= 0.0) {
    std::snprintf(tmp, sizeof(tmp), "  fps=%.1f", hud.fps);
    third += tmp;
  }
  put_text(img, third, 52, colors::kText);
}

}  // namespace

SceneRenderer::SceneRenderer(RenderConfig config) : cfg_(config) {
  // 规范化配置，防止离谱的值（0、负数、NaN）导致创建超大或空图像
  cfg_.width = std::clamp(cfg_.width, 96, 1920);
  cfg_.height = std::clamp(cfg_.height, 96, 1080);
  if (!(cfg_.pixels_per_meter > 0.0) || !std::isfinite(cfg_.pixels_per_meter)) {
    cfg_.pixels_per_meter = 40.0;
  }
}

cv::Point SceneRenderer::world_to_pixel(const Vec2& p, const Vec2& c) const {
  // 世界 y 向上，图像 y 向下，所以 y 方向取反
  return {clamp_px(cfg_.width / 2 + (p.x - c.x) * cfg_.pixels_per_meter),
          clamp_px(cfg_.height / 2 - (p.y - c.y) * cfg_.pixels_per_meter)};
}

cv::Mat SceneRenderer::render(const SceneView& view, const HudInfo& hud) const {
  cv::Mat img(cfg_.height, cfg_.width, CV_8UC3, sc(colors::kBackground));
  const Vec2 c = view.pose.position;
  const double h = view.pose.heading;

  // ---- 网格：每 1 米一条线，画面随车移动时有"地面在动"的运动感 ----
  if (cfg_.draw_grid && std::abs(c.x) < 1e6 && std::abs(c.y) < 1e6) {
    const double half_w = 0.5 * cfg_.width / cfg_.pixels_per_meter;
    const double half_h = 0.5 * cfg_.height / cfg_.pixels_per_meter;
    for (int k = static_cast<int>(std::floor(c.x - half_w)); k <= static_cast<int>(std::ceil(c.x + half_w)); ++k) {
      const int x = world_to_pixel(Vec2{static_cast<double>(k), c.y}, c).x;
      cv::line(img, {x, 0}, {x, cfg_.height - 1}, sc(colors::kGrid), 1, cv::LINE_8);
    }
    for (int k = static_cast<int>(std::floor(c.y - half_h)); k <= static_cast<int>(std::ceil(c.y + half_h)); ++k) {
      const int y = world_to_pixel(Vec2{c.x, static_cast<double>(k)}, c).y;
      cv::line(img, {0, y}, {cfg_.width - 1, y}, sc(colors::kGrid), 1, cv::LINE_8);
    }
  }

  // ---- 路径 ----
  if (view.path != nullptr && view.path->size() >= 2) {
    for (std::size_t i = 0; i + 1 < view.path->size(); ++i) {
      cv::line(img, world_to_pixel((*view.path)[i], c), world_to_pixel((*view.path)[i + 1], c),
               sc(colors::kPath), 1, cv::LINE_8);
    }
  }

  // ---- 障碍物（实心圆）----
  if (view.world != nullptr) {
    for (const auto& o : view.world->obstacles) {
      if (!std::isfinite(o.radius)) continue;
      const int r = std::clamp(clamp_px(o.radius * cfg_.pixels_per_meter), 1, kFar);
      cv::circle(img, world_to_pixel(o.center, c), r, sc(colors::kObstacle), cv::FILLED, cv::LINE_8);
    }
  }

  // ---- 雷达：每根射线画一条淡线，命中点画黄点 ----
  if (cfg_.draw_lidar && view.scan != nullptr) {
    const auto& s = *view.scan;
    const cv::Point origin = world_to_pixel(c, c);
    const std::size_t n = std::min(s.count, sim::kMaxBeams);
    for (std::size_t i = 0; i < n; ++i) {
      const double a = h + s.angle_at(i);        // 射线的世界角度 = 车头朝向 + 相对角度
      const double r = s.ranges[i];
      const cv::Point end = world_to_pixel(Vec2{c.x + r * std::cos(a), c.y + r * std::sin(a)}, c);
      cv::line(img, origin, end, sc(colors::kLidarRay), 1, cv::LINE_8);
      if (s.hit(i)) {
        cv::circle(img, end, 2, sc(colors::kLidarHit), cv::FILLED, cv::LINE_8);
      }
    }
  }

  // ---- 车：三角形，尖角是车头 ----
  // 先在车体坐标系（x 向前、y 向左）里定义三个顶点，再按朝向旋转到世界坐标
  const double ch = std::cos(h), sh = std::sin(h);
  auto place = [&](double lx, double ly) {
    return world_to_pixel(Vec2{c.x + lx * ch - ly * sh, c.y + lx * sh + ly * ch}, c);
  };
  const std::vector<cv::Point> tri{place(0.35, 0.0), place(-0.25, 0.22), place(-0.25, -0.22)};
  cv::fillConvexPoly(img, tri, sc(colors::kVehicle), cv::LINE_8);

  draw_hud(img, hud);
  return img;
}

}  // namespace teledrive::vision
