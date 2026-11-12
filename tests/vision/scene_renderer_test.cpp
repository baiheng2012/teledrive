// 渲染测试：不弹窗，直接检查像素。
// 为了让像素位置可以精确推算，测试里关闭网格和雷达。
// 默认画面 480x360，中心像素 (x=240, y=180)，40 像素/米。
#include <gtest/gtest.h>
#include <teledrive/vision/scene_renderer.hpp>

#include <cmath>
#include <limits>

using teledrive::Vec2;
using teledrive::sim::Circle;
using teledrive::sim::kPi;
using teledrive::sim::Pose;
using teledrive::sim::World;
using namespace teledrive::vision;

namespace {

RenderConfig plain() {
  RenderConfig c;
  c.draw_grid = false;
  c.draw_lidar = false;
  return c;
}

// 注意：cv::Mat::at 的参数顺序是 (行 y, 列 x)
cv::Vec3b px(const cv::Mat& img, int x, int y) { return img.at<cv::Vec3b>(y, x); }

cv::Mat draw(const SceneView& v, const HudInfo& h = {}) { return SceneRenderer(plain()).render(v, h); }

}  // namespace

TEST(WorldToPixel, CenterMapsToImageCenterAndAxesPointTheRightWay) {
  SceneRenderer r;
  const Vec2 c{5.0, 5.0};
  EXPECT_EQ(r.world_to_pixel(c, c), cv::Point(240, 180));
  EXPECT_EQ(r.world_to_pixel(Vec2{6.0, 5.0}, c), cv::Point(280, 180));   // +x → 向右
  EXPECT_EQ(r.world_to_pixel(Vec2{5.0, 6.0}, c), cv::Point(240, 140));   // +y → 向上（图像 y 减小）
}

TEST(WorldToPixel, NonFinitePointsGoFarOffScreenInsteadOfOverflowing) {
  SceneRenderer r;
  const auto p = r.world_to_pixel(Vec2{std::numeric_limits<double>::quiet_NaN(), 1e300}, Vec2{});
  EXPECT_LT(p.x, -1000);
}

TEST(SceneRenderer, ProducesImageOfConfiguredSize) {
  const auto img = draw(SceneView{});
  EXPECT_EQ(img.cols, 480);
  EXPECT_EQ(img.rows, 360);
  EXPECT_EQ(img.type(), CV_8UC3);
  EXPECT_EQ(px(img, 479, 359), colors::kBackground);   // 右下角是背景色
}

TEST(SceneRenderer, DrawsVehicleAtImageCenter) {
  EXPECT_EQ(px(draw(SceneView{}), 240, 180), colors::kVehicle);
}

// 车头朝向：heading=0 车头朝右；heading=π/2 车头朝上（图像里 y 减小的方向）
TEST(SceneRenderer, VehicleNoseFollowsHeading) {
  SceneView v;
  const auto east = draw(v);
  EXPECT_EQ(px(east, 245, 180), colors::kVehicle);       // 车头一侧（右）有车身
  EXPECT_EQ(px(east, 240, 170), colors::kBackground);    // 上方是空的

  v.pose.heading = kPi / 2;
  const auto north = draw(v);
  EXPECT_EQ(px(north, 240, 175), colors::kVehicle);      // 车头朝上，上方有车身
  EXPECT_EQ(px(north, 250, 180), colors::kBackground);   // 右侧是空的
}

TEST(SceneRenderer, DrawsObstacleAtCorrectPlace) {
  World w;
  w.obstacles.push_back(Circle{Vec2{3.0, 0.0}, 0.5});   // 前方 3m → 右移 120 像素
  SceneView v;
  v.world = &w;
  const auto img = draw(v);
  EXPECT_EQ(px(img, 360, 180), colors::kObstacle);
  EXPECT_EQ(px(img, 360, 120), colors::kBackground);     // 障碍物以外还是背景
}

TEST(SceneRenderer, DrawsPathSegment) {
  const std::vector<Vec2> path{Vec2{-1.0, 1.5}, Vec2{1.0, 1.5}};   // 车前上方 1.5m 处的横线
  SceneView v;
  v.path = &path;
  EXPECT_EQ(px(draw(v), 240, 120), colors::kPath);                  // y = 180 - 1.5*40 = 120
}

TEST(SceneRenderer, ViewFollowsTheVehicle) {
  World w;
  w.obstacles.push_back(Circle{Vec2{13.0, 10.0}, 0.5});
  SceneView v;
  v.world = &w;
  v.pose.position = Vec2{10.0, 10.0};     // 车在 (10,10)，障碍物仍在它正前方 3m
  EXPECT_EQ(px(draw(v), 360, 180), colors::kObstacle);
}

TEST(SceneRenderer, HudChangesWithContent) {
  HudInfo a, b;
  a.state = "IDLE";
  b.state = "DRIVING";
  EXPECT_GT(cv::norm(draw(SceneView{}, a), draw(SceneView{}, b), cv::NORM_L1), 0.0);
}

// HUD 只占上方 64 行，不能盖住下面的场景
TEST(SceneRenderer, HudStaysInTopBar) {
  HudInfo h;
  h.note = "OBSTACLE-LIMIT";
  h.latency_ms = 42.0;
  h.fps = 10.0;
  const auto with = draw(SceneView{}, h);
  const auto without = draw(SceneView{});
  const cv::Rect below(0, 64, 480, 360 - 64);
  EXPECT_EQ(cv::norm(with(below), without(below), cv::NORM_L1), 0.0);
}

// 脏输入：不能崩溃，尺寸仍正确
TEST(SceneRenderer, SurvivesNonFiniteAndHugeInputs) {
  World w;
  w.obstacles.push_back(Circle{Vec2{1e12, -1e12}, 1e9});
  w.obstacles.push_back(Circle{Vec2{std::nan(""), 0.0}, std::nan("")});
  SceneView v;
  v.world = &w;
  v.pose.position = Vec2{std::nan(""), 0.0};
  v.pose.heading = std::numeric_limits<double>::infinity();
  const auto img = SceneRenderer().render(v, HudInfo{});     // 默认配置：网格雷达都开
  EXPECT_EQ(img.cols, 480);
  EXPECT_EQ(img.rows, 360);
}
