#pragma once
// 车辆端视频发送器：渲染 -> JPEG -> 分片 -> 经"视频端口"发出。
//
// 用法（车辆端主循环，每个周期）：
//   sender.poll();                       // 收 hello，学习操作端地址
//   if (sender.due()) {                  // 到点了且知道发给谁
//     ...组装 SceneView / HudInfo...
//     sender.send_frame(view, hud);
//   }
// 拆成 due() 和 send_frame() 两步，是为了没到点时不必白白准备场景数据。
//
// 时间来自注入的 IClock，所以"每 100ms 一帧"可以用 FakeClock 测试。
// 只有"耗时统计"用真实时钟：测的就是真实 CPU 耗时，没法用假时钟。

#include <cstdint>
#include <optional>
#include <vector>

#include "teledrive/common/clock.hpp"
#include "teledrive/transport/transport.hpp"
#include "teledrive/vision/scene_renderer.hpp"

namespace teledrive::vision {

struct VideoSenderConfig {
  Duration period{std::chrono::milliseconds(100)};   // 10 fps
  int jpeg_quality{70};
  RenderConfig render{};
};

struct VideoSenderStats {
  std::uint64_t frames_sent{0};
  std::uint64_t packets_sent{0};
  std::uint64_t send_failures{0};
  std::uint64_t encode_failures{0};
  double render_ms_last{0.0}, render_ms_max{0.0};   // 渲染耗时
  double encode_ms_last{0.0}, encode_ms_max{0.0};   // JPEG 压缩耗时
};

class VideoSender {
 public:
  VideoSender(transport::ITransport& transport, const IClock& clock, VideoSenderConfig config = {});

  // 收取 hello 数据报；非 hello 的数据一律忽略。
  void poll();

  // 是否该发下一帧：已知对端，并且距上一帧已满一个周期。
  bool due() const;

  // 渲染并发送一帧。没有对端时返回 false。
  bool send_frame(const SceneView& view, const HudInfo& hud);

  bool has_peer() const { return peer_.has_value(); }
  const VideoSenderStats& stats() const { return stats_; }

 private:
  transport::ITransport& transport_;
  const IClock& clock_;
  VideoSenderConfig cfg_;
  SceneRenderer renderer_;
  TimePoint start_;                       // 拍摄时间戳 = 相对此刻的毫秒数
  std::optional<transport::Endpoint> peer_;
  std::optional<TimePoint> last_send_;
  std::uint16_t frame_id_{0};
  std::vector<std::uint8_t> jpeg_;        // 复用缓冲区，避免每帧重新分配
  VideoSenderStats stats_{};
};

}  // namespace teledrive::vision
