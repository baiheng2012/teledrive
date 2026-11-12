#pragma once
// 把仿真世界渲染成"车载摄像头"俯视画面（以车为中心，世界 +x 朝右，+y 朝上）。
//
// 绘制顺序（后画的盖住先画的）：背景 → 网格 → 路径 → 障碍物 → 雷达 → 车 → HUD
//
// 图像坐标：左上角为原点，x 向右，y 向下；世界坐标 y 向上，所以转换时 y 要取反。

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "teledrive/common/vec2.hpp"
#include "teledrive/sim/diff_drive.hpp"
#include "teledrive/sim/lidar.hpp"

namespace teledrive::vision {

// OpenCV 的颜色顺序是 BGR（蓝、绿、红），不是 RGB。
// 放在头文件里，是为了让测试能直接比对像素颜色。
namespace colors {
inline const cv::Vec3b kBackground{30, 30, 30};
inline const cv::Vec3b kGrid{48, 48, 48};
inline const cv::Vec3b kPath{255, 160, 0};      // 浅蓝
inline const cv::Vec3b kObstacle{60, 60, 230};  // 红
inline const cv::Vec3b kVehicle{80, 220, 80};   // 绿
inline const cv::Vec3b kLidarRay{55, 75, 55};
inline const cv::Vec3b kLidarHit{0, 220, 255};  // 黄
inline const cv::Vec3b kHudBar{15, 15, 15};
inline const cv::Vec3b kText{240, 240, 240};
inline const cv::Vec3b kTextFault{70, 70, 255};
}  // namespace colors

struct RenderConfig {
  int width{480};
  int height{360};
  double pixels_per_meter{40.0};
  bool draw_grid{true};
  bool draw_lidar{true};
};

// 要画什么：指针可以为空，表示"没有这一项"。不拥有数据，只在 render() 调用期间使用。
struct SceneView {
  sim::Pose pose{};
  const sim::World* world{nullptr};
  const std::vector<Vec2>* path{nullptr};
  const sim::LidarScan* scan{nullptr};
};

// HUD（抬头显示）要显示的文字信息
struct HudInfo {
  std::string state{"IDLE"};
  std::string mode{"MANUAL"};
  std::string note{};            // 例如 "OBSTACLE-LIMIT"
  double speed{0.0};
  double battery_pct{100.0};
  double latency_ms{-1.0};       // < 0 表示不显示
  double fps{-1.0};              // < 0 表示不显示
};

class SceneRenderer {
 public:
  explicit SceneRenderer(RenderConfig config = {});

  // 渲染一帧，返回 CV_8UC3 的 BGR 图像。对任何输入（包括 NaN 位姿）都不会崩溃。
  cv::Mat render(const SceneView& view, const HudInfo& hud) const;

  // 世界坐标 → 像素坐标。view_center 是画面中心对应的世界坐标（即车的位置）。
  cv::Point world_to_pixel(const Vec2& p, const Vec2& view_center) const;

  const RenderConfig& config() const { return cfg_; }

 private:
  RenderConfig cfg_;
};

}  // namespace teledrive::vision
