#pragma once
// JPEG 编解码。失败用返回值表示，不抛异常（OpenCV 的异常在这里被捕获）。

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace teledrive::vision {

// 把 BGR 图像压成 JPEG。quality 取值 1~100，超出会被夹住。
// 失败条件：图像为空、不是 3 通道 8 位、或压缩结果超过视频协议的单帧上限。
bool encode_jpeg(const cv::Mat& image, int quality, std::vector<std::uint8_t>& out);

// 解压 JPEG。数据来自网络，不可信：空数据、超长数据、损坏数据都返回 false。
bool decode_jpeg(const std::uint8_t* data, std::size_t len, cv::Mat& out);

}  // namespace teledrive::vision
