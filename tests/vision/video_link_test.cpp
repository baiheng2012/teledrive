// 视频链路测试：VideoSender ⇄ MockTransport ⇄ VideoReceiver，时间由 FakeClock 控制。
// 最后一个测试验证"视频丢光，控制链路完全不受影响"。
#include <gtest/gtest.h>
#include <teledrive/node/operator_session.hpp>
#include <teledrive/node/vehicle_node.hpp>
#include <teledrive/transport/mock_transport.hpp>
#include <teledrive/video/hello.hpp>
#include <teledrive/video/video_packet.hpp>
#include <teledrive/vision/video_receiver.hpp>
#include <teledrive/vision/video_sender.hpp>

using namespace std::chrono_literals;
using namespace teledrive;
namespace tr = teledrive::transport;
namespace proto = teledrive::protocol;

namespace {

const tr::Endpoint kSendEp{0x7F000001u, 9002};   // 车辆端视频 socket
const tr::Endpoint kRecvEp{0x7F000001u, 9003};   // 操作端视频 socket
const tr::Endpoint kStranger{0x0A000009u, 5555};

class VideoLink : public ::testing::Test {
 protected:
  FakeClock clock;
  tr::MockTransport send_t{kSendEp};
  tr::MockTransport recv_t{kRecvEp};
  vision::VideoSender sender{send_t, clock};
  vision::VideoReceiver receiver{recv_t, clock, kSendEp};

  void SetUp() override {
    send_t.connect(&recv_t);
    recv_t.connect(&send_t);
  }

  // 一个 20ms 周期：操作端发 hello → 车辆端学习地址、到点就发帧 → 操作端收帧
  void tick() {
    receiver.send_hello_if_due();
    sender.poll();
    if (sender.due()) sender.send_frame(vision::SceneView{}, vision::HudInfo{});
    receiver.poll();
    clock.advance(20ms);
  }
  void ticks(int n) { for (int i = 0; i < n; ++i) tick(); }
};

}  // namespace

// ---------- 握手 ----------

TEST_F(VideoLink, NoHelloMeansNoFrames) {
  sender.poll();
  EXPECT_FALSE(sender.has_peer());
  EXPECT_FALSE(sender.due());
  EXPECT_FALSE(sender.send_frame(vision::SceneView{}, vision::HudInfo{}));
  EXPECT_EQ(sender.stats().frames_sent, 0u);
}

TEST_F(VideoLink, HelloTeachesSenderWhereToSend) {
  receiver.send_hello_if_due();
  sender.poll();
  EXPECT_TRUE(sender.has_peer());
  EXPECT_TRUE(sender.due());
}

TEST_F(VideoLink, SenderIgnoresNonHelloDatagrams) {
  const std::uint8_t junk[4] = {1, 2, 3, 4};
  const std::uint8_t longer[5] = {0x54, 0x56, 0x48, 0x49, 0x00};   // hello 多一个字节
  send_t.inject(kStranger, junk, sizeof(junk));
  send_t.inject(kStranger, longer, sizeof(longer));
  sender.poll();
  EXPECT_FALSE(sender.has_peer());
}

TEST_F(VideoLink, HelloIsRateLimitedAndRepeats) {
  receiver.send_hello_if_due();
  receiver.send_hello_if_due();
  EXPECT_EQ(recv_t.sent_count(), 1u);       // 同一时刻只发一次
  clock.advance(999ms);
  receiver.send_hello_if_due();
  EXPECT_EQ(recv_t.sent_count(), 1u);
  clock.advance(1ms);
  receiver.send_hello_if_due();
  EXPECT_EQ(recv_t.sent_count(), 2u);       // 满 1 秒再发一次（车辆端重启后靠它恢复）
}

// ---------- 帧的传输 ----------

TEST_F(VideoLink, FrameTravelsEndToEnd) {
  tick();
  EXPECT_EQ(sender.stats().frames_sent, 1u);
  EXPECT_GT(sender.stats().packets_sent, 1u);
  ASSERT_EQ(receiver.stats().frames, 1u);
  ASSERT_TRUE(receiver.has_frame());
  EXPECT_EQ(receiver.latest_frame().cols, 480);
  EXPECT_EQ(receiver.latest_frame().rows, 360);

  EXPECT_TRUE(receiver.take_new_frame());
  EXPECT_FALSE(receiver.take_new_frame());   // 标志取走后清除
}

TEST_F(VideoLink, FramesAreSentOncePerPeriod) {
  ticks(5);                                   // t = 0..80ms
  EXPECT_EQ(sender.stats().frames_sent, 1u);
  tick();                                     // t = 100ms
  EXPECT_EQ(sender.stats().frames_sent, 2u);
}

TEST_F(VideoLink, RecordsRenderAndEncodeTimings) {
  tick();
  EXPECT_GT(sender.stats().render_ms_last, 0.0);
  EXPECT_GT(sender.stats().encode_ms_last, 0.0);
  EXPECT_GE(sender.stats().encode_ms_max, sender.stats().encode_ms_last);
}

// ---------- 不可信输入 ----------

TEST_F(VideoLink, ReceiverIgnoresPacketsFromOtherSources) {
  const std::vector<std::uint8_t> data(100, 7);
  for (const auto& p : video::fragment_frame(1, 0, data.data(), data.size())) {
    recv_t.inject(kStranger, p.data(), p.size());
  }
  receiver.poll();
  EXPECT_EQ(receiver.stats().foreign_packets, 1u);
  EXPECT_EQ(receiver.stats().frames, 0u);
  EXPECT_EQ(receiver.reassembly().completed, 0u);   // 连重组器都没碰到
}

// 分片协议层面完全合法，但内容不是 JPEG：重组成功，解码失败，既不崩溃也不显示
TEST_F(VideoLink, ReceiverCountsUndecodableFrames) {
  const std::vector<std::uint8_t> not_a_jpeg(100, 7);
  for (const auto& p : video::fragment_frame(1, 0, not_a_jpeg.data(), not_a_jpeg.size())) {
    recv_t.inject(kSendEp, p.data(), p.size());
  }
  receiver.poll();
  EXPECT_EQ(receiver.reassembly().completed, 1u);
  EXPECT_EQ(receiver.stats().decode_failures, 1u);
  EXPECT_EQ(receiver.stats().frames, 0u);
  EXPECT_FALSE(receiver.has_frame());
}

// ---------- 帧率与抖动 ----------

TEST_F(VideoLink, SteadyStreamGivesTenFpsAndNoJitter) {
  ticks(100);   // 2 秒，20 帧
  EXPECT_NEAR(receiver.fps(), 10.0, 0.5);
  EXPECT_LT(receiver.jitter_ms(), 1.0);
}

TEST_F(VideoLink, DelayedArrivalRaisesJitter) {
  ticks(50);
  ASSERT_LT(receiver.jitter_ms(), 1.0);

  // 手动发一帧，但让它"晚到" 40ms：到达间隔比拍摄间隔多 40ms
  while (!sender.due()) clock.advance(20ms);
  sender.send_frame(vision::SceneView{}, vision::HudInfo{});
  clock.advance(40ms);
  receiver.poll();

  EXPECT_GT(receiver.jitter_ms(), 3.0);   // 0.2 * 40 = 8（滑动平均后）
}

TEST_F(VideoLink, FpsDropsToZeroAfterStreamStops) {
  ticks(25);
  ASSERT_GT(receiver.fps(), 5.0);
  clock.advance(2s);
  EXPECT_DOUBLE_EQ(receiver.fps(), 0.0);
}

// ---------- 控制面与数据面分离 ----------

namespace {

const tr::Endpoint kOp{0x7F000001u, 9000};
const tr::Endpoint kVeh{0x7F000001u, 9001};

// 同一段操作（arm 后以 0.5 m/s 行驶 5 秒），返回车最终的 x 位置。
// video_on：是否启用视频链路；drop_video：视频包全部被网络丢弃
double drive_and_get_x(bool video_on, bool drop_video) {
  FakeClock clock;
  tr::MockTransport op_t{kOp}, veh_t{kVeh}, vs_t{kSendEp}, vr_t{kRecvEp};
  op_t.connect(&veh_t);
  veh_t.connect(&op_t);
  vs_t.connect(&vr_t);
  vr_t.connect(&vs_t);
  if (drop_video) vs_t.drop_next_sends(1000000);

  node::VehicleNode vehicle(veh_t, clock);
  node::OperatorSession op(op_t, clock, kVeh);
  vision::VideoSender sender(vs_t, clock);
  vision::VideoReceiver receiver(vr_t, clock, kSendEp);

  for (int i = 0; i < 250; ++i) {
    proto::Command c;
    c.flags = proto::kFlagArm;
    c.linear = i < 2 ? 0.0f : 0.5f;
    op.send_command(c);
    vehicle.tick();
    op.poll();

    if (video_on) {
      receiver.send_hello_if_due();
      sender.poll();
      if (sender.due()) {
        sender.send_frame(vision::SceneView{vehicle.model().state().pose}, vision::HudInfo{});
      }
      receiver.poll();
    }
    clock.advance(20ms);
  }
  return vehicle.model().state().pose.position.x;
}

}  // namespace

TEST(ControlPlaneIndependence, VideoOrItsTotalLossNeverChangesVehicleBehaviour) {
  const double without_video = drive_and_get_x(false, false);
  const double with_video = drive_and_get_x(true, false);
  const double video_lost = drive_and_get_x(true, true);

  EXPECT_GT(without_video, 1.5);                    // 确实开出去了
  EXPECT_DOUBLE_EQ(with_video, without_video);      // 开着视频，行为完全一致
  EXPECT_DOUBLE_EQ(video_lost, without_video);      // 视频全丢，行为也完全一致
}
