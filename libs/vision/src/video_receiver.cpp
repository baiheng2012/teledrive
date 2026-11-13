#include "teledrive/vision/video_receiver.hpp"

#include <chrono>
#include <cmath>

#include "teledrive/video/hello.hpp"
#include "teledrive/vision/jpeg_codec.hpp"

namespace teledrive::vision {

namespace {
constexpr double kAlpha = 0.2;   // 滑动平均系数：新值占 20%
constexpr auto kHelloInterval = std::chrono::seconds(1);
// 一帧约 17 个分片；每个周期最多处理 256 个，足够 10 帧的突发，又不会无限循环
constexpr int kMaxRxPerPoll = 256;
}  // namespace

VideoReceiver::VideoReceiver(transport::ITransport& transport, const IClock& clock,
                             transport::Endpoint vehicle)
    : transport_(transport), clock_(clock), vehicle_(vehicle) {}

void VideoReceiver::send_hello_if_due() {
  const TimePoint now = clock_.now();
  if (last_hello_ && (now - *last_hello_) < kHelloInterval) return;
  last_hello_ = now;
  transport_.send(vehicle_, video::kHello.data(), video::kHello.size());
}

void VideoReceiver::poll() {
  transport::Datagram d;
  for (int i = 0; i < kMaxRxPerPoll && transport_.receive(d); ++i) {
    // 只信任来自车辆的数据报
    if (d.from != vehicle_) {
      ++stats_.foreign_packets;
      continue;
    }
    video::VideoFrame f;
    if (!reassembler_.push(d.data.data(), d.size, f)) continue;   // 还没收齐，或是坏包

    cv::Mat img;
    if (!decode_jpeg(f.data.data(), f.data.size(), img)) {
      ++stats_.decode_failures;
      continue;
    }
    on_frame(f, std::move(img));
  }
}

void VideoReceiver::on_frame(const video::VideoFrame& f, cv::Mat&& img) {
  const TimePoint now = clock_.now();
  if (have_prev_) {
    const double arrival_dt =
        std::chrono::duration<double, std::milli>(now - prev_arrival_).count();
    // u32 无符号相减：即使毫秒计数回绕，差值依然正确
    const double capture_dt = static_cast<double>(
        static_cast<std::uint32_t>(f.capture_ms - prev_capture_ms_));

    interval_ms_ = have_interval_ ? interval_ms_ + kAlpha * (arrival_dt - interval_ms_) : arrival_dt;
    have_interval_ = true;
    jitter_ms_ += kAlpha * (std::abs(arrival_dt - capture_dt) - jitter_ms_);
  }
  have_prev_ = true;
  prev_arrival_ = now;
  prev_capture_ms_ = f.capture_ms;

  frame_ = std::move(img);
  new_frame_ = true;
  ++stats_.frames;
}

bool VideoReceiver::take_new_frame() {
  const bool r = new_frame_;
  new_frame_ = false;
  return r;
}

double VideoReceiver::fps() const {
  if (!have_prev_ || !have_interval_ || interval_ms_ <= 0.0) return 0.0;
  if ((clock_.now() - prev_arrival_) > std::chrono::seconds(1)) return 0.0;   // 断流了
  return 1000.0 / interval_ms_;
}

}  // namespace teledrive::vision
