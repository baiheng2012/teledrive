// 渲染一帧演示画面并保存为 PNG，同时打印各环节耗时和数据量。
// 用法：render_demo [输出文件，默认 docs/vision_demo.png]
#include <chrono>
#include <cstdio>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "teledrive/video/video_packet.hpp"
#include "teledrive/vision/jpeg_codec.hpp"
#include "teledrive/vision/scene_renderer.hpp"

int main(int argc, char** argv) {
  using namespace teledrive;
  const char* out_path = argc > 1 ? argv[1] : "docs/vision_demo.png";

  // 与 vehicle_sim 相同的场景：前方 5m 一个障碍物，方形路径；车开到了路径中段
  sim::World world;
  world.obstacles.push_back(sim::Circle{Vec2{5.0, 0.0}, 0.5});
  const std::vector<Vec2> path{{0, 0}, {3, 0}, {3, 3}, {0, 3}, {0, 0.6}};
  sim::Pose pose;
  pose.position = Vec2{2.4, 0.15};
  pose.heading = 0.1;

  const sim::Lidar lidar;
  const auto scan = lidar.scan(world, pose);

  vision::SceneView view;
  view.pose = pose;
  view.world = &world;
  view.path = &path;
  view.scan = &scan;

  vision::HudInfo hud;
  hud.state = "DRIVING";
  hud.mode = "AUTO";
  hud.note = "OBSTACLE-LIMIT";
  hud.speed = 0.6;
  hud.battery_pct = 97;
  hud.latency_ms = 42;
  hud.fps = 10;

  using Clock = std::chrono::steady_clock;
  auto ms = [](Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };

  const auto t0 = Clock::now();
  const cv::Mat img = vision::SceneRenderer().render(view, hud);
  const auto t1 = Clock::now();
  std::vector<std::uint8_t> jpeg;
  vision::encode_jpeg(img, 70, jpeg);
  const auto t2 = Clock::now();

  cv::imwrite(out_path, img);
  const auto packets = video::fragment_frame(1, 0, jpeg.data(), jpeg.size());
  std::printf("render: %.2f ms | jpeg encode: %.2f ms\n", ms(t0, t1), ms(t1, t2));
  std::printf("raw %zu bytes -> jpeg %zu bytes (%.1fx) -> %zu UDP packets\n",
              img.total() * img.channels(), jpeg.size(),
              static_cast<double>(img.total() * img.channels()) / static_cast<double>(jpeg.size()),
              packets.size());
  std::printf("at 10 fps: %.0f KB/s\n", static_cast<double>(jpeg.size()) * 10 / 1024.0);
  std::printf("saved %s\n", out_path);
  return 0;
}
