#pragma once
// 操作端 -> 车辆端视频端口的"握手"数据报。
// 作用：让车辆端知道视频该发往哪个地址（操作端的视频 socket 端口是系统随机分配的）。
// 操作端每秒发一次，所以车辆端重启后能自动重新学到地址。
// 已知局限：没有认证，同网段任何人发这 4 个字节都能把视频引到自己那里。

#include <array>
#include <cstdint>

namespace teledrive::video {

inline constexpr std::array<std::uint8_t, 4> kHello{0x54, 0x56, 0x48, 0x49};  // "TVHI"

}  // namespace teledrive::video
