#include "teledrive/vision/jpeg_codec.hpp"

#include <algorithm>

#include <opencv2/imgcodecs.hpp>

#include "teledrive/video/video_packet.hpp"

namespace teledrive::vision {

bool encode_jpeg(const cv::Mat& image, int quality, std::vector<std::uint8_t>& out) {
  out.clear();
  if (image.empty() || image.type() != CV_8UC3) {
    return false;
  }
  const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, std::clamp(quality, 1, 100)};
  try {
    if (!cv::imencode(".jpg", image, out, params)) {
      out.clear();
      return false;
    }
  } catch (const cv::Exception&) {   // OpenCV 用异常报错；我们的约定是返回值，在边界处转换
    out.clear();
    return false;
  }
  if (out.size() > video::kMaxFrameBytes) {   // 一帧太大，分片协议装不下
    out.clear();
    return false;
  }
  return true;
}

bool decode_jpeg(const std::uint8_t* data, std::size_t len, cv::Mat& out) {
  // 先拦截明显不合理的输入，再交给解码器：数据来自网络，不可信
  if (data == nullptr || len == 0 || len > video::kMaxFrameBytes) {
    return false;
  }
  try {
    // 复制一份再解码：避免为了适配 OpenCV 接口而去掉 const
    const std::vector<std::uint8_t> copy(data, data + len);
    cv::Mat img = cv::imdecode(copy, cv::IMREAD_COLOR);
    if (img.empty()) {
      return false;
    }
    out = std::move(img);
    return true;
  } catch (const cv::Exception&) {
    return false;
  }
}

}  // namespace teledrive::vision
