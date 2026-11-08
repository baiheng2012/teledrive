#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <variant>

namespace teledrive::protocol {

// ---------- 协议常量 ----------
inline constexpr std::uint16_t kMagic = 0x5444;       // "TD"
inline constexpr std::uint8_t  kVersion = 1;
inline constexpr std::size_t   kHeaderSize = 8;       // magic2 + ver1 + type1 + seq2 + len2
inline constexpr std::size_t   kCrcSize = 2;
inline constexpr std::size_t   kMaxPayload = 64;      // 载荷上限：拒绝异常大的长度字段
inline constexpr std::size_t   kMaxFrameSize = kHeaderSize + kMaxPayload + kCrcSize;

// 消息类型。enum class：强类型，不会隐式转成 int，避免误用。
enum class MsgType : std::uint8_t {
  Heartbeat = 1,
  Command   = 2,
  Telemetry = 3,
};

// ---------- 三种消息 ----------
struct Heartbeat {
  std::uint32_t timestamp_ms{0};
};

// Command.flags 的位定义
inline constexpr std::uint8_t kFlagEStop = 0x01;  // bit0: 急停请求
inline constexpr std::uint8_t kFlagArm   = 0x02;  // bit1: 请求上电/arm
inline constexpr std::uint8_t kFlagReset = 0x04;  // bit2: 请求清除故障（Fault -> Idle）

struct Command {
  float linear{0.0f};    // 期望线速度 m/s
  float angular{0.0f};   // 期望角速度 rad/s
  std::uint8_t flags{0};
};

// 车辆状态，会和下一阶段的安全状态机对应
enum class VehicleState : std::uint8_t {
  Idle = 0, Armed = 1, Driving = 2, Fault = 3,
};

struct Telemetry {
  float x{0.0f}, y{0.0f};     // 位置 m
  float heading{0.0f};        // 朝向 rad
  float speed{0.0f};          // 当前速度 m/s
  VehicleState state{VehicleState::Idle};
  std::uint8_t battery_pct{100};
};

// variant：一个变量可以是三种消息中的任意一种（类型安全的 union）
using Payload = std::variant<Heartbeat, Command, Telemetry>;

// 解码出来的一整帧：头部信息 + 载荷
struct Frame {
  std::uint16_t seq{0};
  Payload payload;
};

// ---------- 编码 ----------
// 固定大小的缓冲区，避免堆分配（控制循环里不 new / 不用 vector）。
struct EncodedFrame {
  std::array<std::uint8_t, kMaxFrameSize> data{};
  std::size_t size{0};   // 实际有效字节数
};

// 把载荷编码成完整一帧（含头和 CRC）。
EncodedFrame encode(std::uint16_t seq, const Payload& payload);

// ---------- 解码 ----------
// 所有可能的失败原因。坏包是网络上的常态，所以用错误码而不是异常。
enum class DecodeError : std::uint8_t {
  None = 0,
  TooShort,        // 连头部+CRC 的最小长度都不够
  BadMagic,        // 帧头不对
  BadVersion,      // 协议版本不支持
  BadLength,       // length 字段超限，或与实际收到的字节数不一致
  BadCrc,          // 校验失败（数据被破坏）
  UnknownType,     // 未知消息类型
  BadPayloadSize,  // 载荷大小与该类型应有的大小不符
};

struct DecodeResult {
  DecodeError error{DecodeError::None};
  Frame frame{};
  bool ok() const { return error == DecodeError::None; }
};

DecodeResult decode(const std::uint8_t* data, std::size_t len);

}  // namespace teledrive::protocol
