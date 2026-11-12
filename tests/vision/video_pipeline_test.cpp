// 整条视频链路（不含真实网络）：渲染 → JPEG → 分片 → MockTransport → 重组 → 解码
#include <gtest/gtest.h>
#include <teledrive/transport/mock_transport.hpp>
#include <teledrive/video/video_packet.hpp>
#include <teledrive/vision/jpeg_codec.hpp>
#include <teledrive/vision/scene_renderer.hpp>

using namespace teledrive;
namespace tr = teledrive::transport;

namespace {
const tr::Endpoint kVeh{0x7F000001u, 9002}, kOp{0x7F000001u, 9003};

double mean_abs_diff(const cv::Mat& a, const cv::Mat& b) {
  return cv::norm(a, b, cv::NORM_L1) / static_cast<double>(a.total() * a.channels());
}

struct Pipeline {
  tr::MockTransport vehicle{kVeh};
  tr::MockTransport operator_side{kOp};
  video::FrameReassembler reassembler;
  cv::Mat scene;
  std::vector<std::uint8_t> jpeg;

  Pipeline() {
    vehicle.connect(&operator_side);
    scene = vision::SceneRenderer().render(vision::SceneView{}, vision::HudInfo{});
    EXPECT_TRUE(vision::encode_jpeg(scene, 70, jpeg));
  }

  // 车辆端：把当前画面作为第 id 帧发出
  void send_frame(std::uint16_t id, std::uint32_t ms) {
    for (const auto& p : video::fragment_frame(id, ms, jpeg.data(), jpeg.size())) {
      vehicle.send(kOp, p.data(), p.size());
    }
  }

  // 操作端：收取所有数据报并重组，返回是否得到了完整的一帧
  bool receive(video::VideoFrame& out) {
    bool got = false;
    tr::Datagram d;
    while (operator_side.receive(d)) got = reassembler.push(d.data.data(), d.size, out) || got;
    return got;
  }
};
}  // namespace

TEST(VideoPipeline, FrameSurvivesTheWholeChain) {
  Pipeline p;
  p.send_frame(1, 5000);

  video::VideoFrame f;
  ASSERT_TRUE(p.receive(f));
  EXPECT_EQ(f.capture_ms, 5000u);

  cv::Mat shown;
  ASSERT_TRUE(vision::decode_jpeg(f.data.data(), f.data.size(), shown));
  EXPECT_EQ(shown.size(), p.scene.size());
  EXPECT_LT(mean_abs_diff(shown, p.scene), 6.0);
}

// 丢一个分片：这一帧不显示（不会出现花屏的残缺图像），下一帧恢复正常
TEST(VideoPipeline, LostFragmentSkipsFrameThenRecovers) {
  Pipeline p;
  ASSERT_GT(video::fragment_frame(1, 0, p.jpeg.data(), p.jpeg.size()).size(), 1u);  // 确实是多片

  p.vehicle.drop_next_sends(1);   // 第 1 帧的第 1 个分片丢了
  p.send_frame(1, 100);
  video::VideoFrame f;
  EXPECT_FALSE(p.receive(f));

  p.send_frame(2, 200);
  ASSERT_TRUE(p.receive(f));
  EXPECT_EQ(f.frame_id, 2);
  EXPECT_EQ(p.reassembler.stats().dropped_incomplete, 1u);
}
