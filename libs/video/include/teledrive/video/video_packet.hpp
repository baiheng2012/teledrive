#pragma once
// 视频分包协议：把一帧 JPEG 拆成多个 UDP 数据报，收端重组。
//
// 设计原则：
//   1) 与控制协议完全独立（不同 magic、不同端口），视频拥塞不影响控制指令
//   2) "新帧优先"：重组器同一时刻只组装一帧，遇到更新的帧就放弃旧帧，不重传。
//      对实时视频来说，迟到的画面比丢掉的画面更没用
//   3) 不依赖 OpenCV：纯字节处理，所以可以独立测试
//
// 视频路径不在 50Hz 控制循环的"硬实时"部分里，这里允许使用 std::vector。

#include <cstddef>
#include <cstdint>
#include <vector>

namespace teledrive::video {

inline constexpr std::uint16_t kVideoMagic = 0x5654;   // "TV"
inline constexpr std::uint8_t  kVideoVersion = 1;
inline constexpr std::size_t   kVideoHeaderSize = 13;
inline constexpr std::size_t   kVideoCrcSize = 2;
inline constexpr std::size_t   kMaxFragmentPayload = 1200;  // 整包约 1215 字节 < 以太网 MTU 1500
inline constexpr std::size_t   kMaxFragments = 255;         // frag_count 是 u8
inline constexpr std::size_t   kMaxFrameBytes = kMaxFragments * kMaxFragmentPayload;  // 约 306 KB

using Packet = std::vector<std::uint8_t>;

// 把一帧数据拆成若干数据包。数据为空或超过 kMaxFrameBytes 时返回空列表。
std::vector<Packet> fragment_frame(std::uint16_t frame_id, std::uint32_t capture_ms,
                                   const std::uint8_t* data, std::size_t len);

// 帧号 a 是否比 b 更新（16 位回绕比较，与控制协议的序列号同样的做法）
inline bool frame_id_newer(std::uint16_t a, std::uint16_t b) {
  return static_cast<std::int16_t>(static_cast<std::uint16_t>(a - b)) > 0;
}

struct VideoFrame {
  std::uint16_t frame_id{0};
  std::uint32_t capture_ms{0};
  std::vector<std::uint8_t> data;   // 重组后的完整 JPEG
};

struct ReassemblerStats {
  std::uint64_t completed{0};            // 成功重组的帧数
  std::uint64_t dropped_incomplete{0};   // 没收齐就被新帧顶掉的帧数（丢包的代价）
  std::uint64_t bad_packets{0};          // 格式错误、CRC 错误等
  std::uint64_t stale_packets{0};        // 旧帧的迟到分片、重复分片
};

class FrameReassembler {
 public:
  // 喂入一个收到的数据报。如果它恰好补全了一帧，返回 true 并填好 out。
  bool push(const std::uint8_t* data, std::size_t len, VideoFrame& out);

  const ReassemblerStats& stats() const { return stats_; }

 private:
  void start_frame(std::uint16_t frame_id, std::uint8_t count, std::uint32_t capture_ms);

  // 正在组装的帧
  bool have_current_{false};
  std::uint16_t cur_id_{0};
  std::uint8_t cur_count_{0};
  std::uint32_t cur_capture_ms_{0};
  std::vector<std::vector<std::uint8_t>> parts_;
  std::vector<bool> have_;
  std::size_t received_{0};

  // 最近一次完成的帧号：之后到达的、不比它新的分片一律视为过期
  bool have_last_{false};
  std::uint16_t last_id_{0};

  ReassemblerStats stats_{};
};

}  // namespace teledrive::video
