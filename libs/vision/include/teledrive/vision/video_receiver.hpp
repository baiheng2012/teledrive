#pragma once
// 操作端视频接收器：收片 -> 重组 -> 解码，并统计帧率和抖动。
//
// 抖动 = |相邻两帧的到达间隔 - 相邻两帧的拍摄间隔| 的滑动平均。
// 为什么这样算：两个时钟不同步，capture_ms 和本机时间不能直接相减；
// 但"同一台机器上两个时刻的差"不需要同步，所以两种间隔可以相减。

#include <cstdint>

#include <opencv2/core.hpp>

#include "teledrive/common/clock.hpp"
#include "teledrive/transport/transport.hpp"
#include "teledrive/video/video_packet.hpp"

namespace teledrive::vision {

struct VideoReceiverStats {
  std::uint64_t frames{0};            // 成功解码显示的帧数
  std::uint64_t decode_failures{0};   // 重组成功但 JPEG 解码失败
  std::uint64_t foreign_packets{0};   // 来源不是车辆地址的数据报（被忽略）
};

class VideoReceiver {
 public:
  VideoReceiver(transport::ITransport& transport, const IClock& clock,
                transport::Endpoint vehicle);

  // 每秒最多发一次 hello。每个周期调用一次即可。
  void send_hello_if_due();

  // 收取并处理所有已到达的数据报。
  void poll();

  // 自上次调用以来是否有新帧（调用后清除标志）。
  bool take_new_frame();

  bool has_frame() const { return !frame_.empty(); }
  const cv::Mat& latest_frame() const { return frame_; }

  // 最近的帧率。超过 1 秒没收到新帧，返回 0。
  double fps() const;
  double jitter_ms() const { return jitter_ms_; }

  const VideoReceiverStats& stats() const { return stats_; }
  const video::ReassemblerStats& reassembly() const { return reassembler_.stats(); }

 private:
  void on_frame(const video::VideoFrame& f, cv::Mat&& img);

  transport::ITransport& transport_;
  const IClock& clock_;
  transport::Endpoint vehicle_;
  video::FrameReassembler reassembler_;

  cv::Mat frame_;
  bool new_frame_{false};

  std::optional<TimePoint> last_hello_;

  bool have_prev_{false};
  TimePoint prev_arrival_{};
  std::uint32_t prev_capture_ms_{0};
  bool have_interval_{false};
  double interval_ms_{0.0};   // 帧到达间隔的滑动平均
  double jitter_ms_{0.0};

  VideoReceiverStats stats_{};
};

}  // namespace teledrive::vision
