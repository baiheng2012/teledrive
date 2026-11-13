#include "teledrive/vision/video_sender.hpp"

#include <algorithm>
#include <chrono>

#include "teledrive/video/hello.hpp"
#include "teledrive/video/video_packet.hpp"
#include "teledrive/vision/jpeg_codec.hpp"

namespace teledrive::vision {

namespace {
using ProfileClock = std::chrono::steady_clock;   // 只用于测耗时
double ms_between(ProfileClock::time_point a, ProfileClock::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}
}  // namespace

VideoSender::VideoSender(transport::ITransport& transport, const IClock& clock,
                         VideoSenderConfig config)
    : transport_(transport),
      clock_(clock),
      cfg_(config),
      renderer_(config.render),
      start_(clock.now()) {}

void VideoSender::poll() {
  // 每次最多处理 16 个，防止有人灌包拖慢控制循环（和 VehicleNode 同样的原则）
  transport::Datagram d;
  for (int i = 0; i < 16 && transport_.receive(d); ++i) {
    const bool is_hello =
        d.size == video::kHello.size() &&
        std::equal(video::kHello.begin(), video::kHello.end(), d.data.begin());
    if (is_hello) {
      peer_ = d.from;
    }
  }
}

bool VideoSender::due() const {
  if (!peer_) return false;
  return !last_send_ || (clock_.now() - *last_send_) >= cfg_.period;
}

bool VideoSender::send_frame(const SceneView& view, const HudInfo& hud) {
  if (!peer_) return false;
  const TimePoint now = clock_.now();
  last_send_ = now;   // 失败也记录：否则编码持续失败时会每个周期都重试，拖垮控制循环

  const auto t0 = ProfileClock::now();
  const cv::Mat img = renderer_.render(view, hud);
  const auto t1 = ProfileClock::now();
  const bool ok = encode_jpeg(img, cfg_.jpeg_quality, jpeg_);
  const auto t2 = ProfileClock::now();

  stats_.render_ms_last = ms_between(t0, t1);
  stats_.encode_ms_last = ms_between(t1, t2);
  stats_.render_ms_max = std::max(stats_.render_ms_max, stats_.render_ms_last);
  stats_.encode_ms_max = std::max(stats_.encode_ms_max, stats_.encode_ms_last);

  if (!ok) {
    ++stats_.encode_failures;
    return false;
  }

  const auto capture_ms = static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - start_).count());
  const auto packets = video::fragment_frame(frame_id_++, capture_ms, jpeg_.data(), jpeg_.size());
  for (const auto& p : packets) {
    if (transport_.send(*peer_, p.data(), p.size())) {
      ++stats_.packets_sent;
    } else {
      ++stats_.send_failures;
    }
  }
  ++stats_.frames_sent;
  return true;
}

}  // namespace teledrive::vision
