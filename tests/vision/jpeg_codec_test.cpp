#include <gtest/gtest.h>
#include <teledrive/vision/jpeg_codec.hpp>
#include <teledrive/vision/scene_renderer.hpp>
#include <teledrive/video/video_packet.hpp>

using namespace teledrive::vision;

namespace {

cv::Mat sample_scene() {
  teledrive::sim::World w;
  w.obstacles.push_back({teledrive::Vec2{3.0, 0.5}, 0.5});
  const std::vector<teledrive::Vec2> path{{0, 0}, {4, 0}, {4, 3}};
  SceneView v;
  v.world = &w;
  v.path = &path;
  HudInfo h;
  h.state = "DRIVING";
  h.latency_ms = 30.0;
  return SceneRenderer().render(v, h);
}

// 平均每个通道值的绝对误差
double mean_abs_diff(const cv::Mat& a, const cv::Mat& b) {
  return cv::norm(a, b, cv::NORM_L1) / static_cast<double>(a.total() * a.channels());
}

}  // namespace

TEST(JpegCodec, RoundTripKeepsSizeAndContent) {
  const auto img = sample_scene();
  std::vector<std::uint8_t> bytes;
  ASSERT_TRUE(encode_jpeg(img, 80, bytes));

  cv::Mat back;
  ASSERT_TRUE(decode_jpeg(bytes.data(), bytes.size(), back));
  EXPECT_EQ(back.size(), img.size());
  EXPECT_EQ(back.type(), CV_8UC3);
  EXPECT_LT(mean_abs_diff(img, back), 6.0);     // JPEG 有损，但整体非常接近
}

TEST(JpegCodec, CompressionIsEffective) {
  const auto img = sample_scene();
  std::vector<std::uint8_t> bytes;
  ASSERT_TRUE(encode_jpeg(img, 70, bytes));
  EXPECT_LT(bytes.size(), img.total() * img.channels() / 5);   // 至少压缩 5 倍
}

TEST(JpegCodec, LowerQualityIsSmaller) {
  const auto img = sample_scene();
  std::vector<std::uint8_t> low, high;
  ASSERT_TRUE(encode_jpeg(img, 30, low));
  ASSERT_TRUE(encode_jpeg(img, 95, high));
  EXPECT_LT(low.size(), high.size());
}

TEST(JpegCodec, EncodeRejectsBadImages) {
  std::vector<std::uint8_t> out;
  EXPECT_FALSE(encode_jpeg(cv::Mat{}, 80, out));                          // 空图
  EXPECT_FALSE(encode_jpeg(cv::Mat(10, 10, CV_8UC1, cv::Scalar(0)), 80, out));   // 灰度图
}

TEST(JpegCodec, DecodeRejectsGarbageAndBadLengths) {
  cv::Mat out;
  const std::uint8_t junk[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  EXPECT_FALSE(decode_jpeg(junk, sizeof(junk), out));
  EXPECT_FALSE(decode_jpeg(nullptr, 10, out));
  EXPECT_FALSE(decode_jpeg(junk, 0, out));
  std::vector<std::uint8_t> huge(teledrive::video::kMaxFrameBytes + 1, 0xFF);
  EXPECT_FALSE(decode_jpeg(huge.data(), huge.size(), out));
}
