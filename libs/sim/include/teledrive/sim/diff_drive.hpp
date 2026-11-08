#pragma once
// 差速驱动车辆的运动学仿真模型。
//
// 约定（所有代码和测试都遵守）：
//   坐标系：x 向前（朝向 0 时），y 向左，朝向 θ 逆时针为正，单位 rad
//   单位：米、秒、弧度
//   内部用 double 计算（积分累积误差小）；只有在生成协议消息时才转成 float
//
// 本类只描述"车怎么动"，不知道网络、不知道安全状态机。
// 它只接收"想要的速度"，是否允许行驶由 safety 层负责。

#include "teledrive/common/vec2.hpp"
#include "teledrive/protocol/messages.hpp"

namespace teledrive::sim {

// 自己定义 π：M_PI 不是 C++ 标准的一部分，在某些编译器上需要额外宏才可用
inline constexpr double kPi = 3.14159265358979323846;

struct Pose {
  Vec2 position{};       // 位置 (m)
  double heading{0.0};   // 朝向 (rad)，始终保持在 (-π, π]
};

// 车辆的物理参数。默认值是一辆"小型工业遥控车"的合理量级。
struct VehicleParams {
  // 物理上限：就算收到更大的目标值，车也跑不出这个速度（与 safety 层的限幅是两道独立防线）
  double max_linear_speed{2.0};    // m/s
  double max_angular_speed{4.0};   // rad/s

  // 加速度限制：速度变化率的上限
  double max_linear_accel{1.0};    // m/s²   加速（速度绝对值变大）
  double max_linear_decel{2.0};    // m/s²   减速（刹车比加速更猛）
  double max_angular_accel{3.0};   // rad/s²
  double max_angular_decel{6.0};   // rad/s²

  // 电量消耗（百分点/秒）：静止也耗电（控制器、传感器），运动额外耗电
  double battery_idle_drain{0.01};          // %/s
  double battery_motion_drain_per_mps{0.05}; // %/s，每 1 m/s 速度额外消耗
};

// 车辆的完整状态（仿真的"真值"）
struct DiffDriveState {
  Pose pose{};
  double linear_velocity{0.0};    // 当前实际线速度 m/s（后退为负）
  double angular_velocity{0.0};   // 当前实际角速度 rad/s
  double battery_pct{100.0};      // 0 ~ 100
};

class DiffDriveModel {
 public:
  explicit DiffDriveModel(VehicleParams params = {}) : params_(params) {}

  // 推进仿真一个时间步。
  //   target：想要的速度（通常来自 SafetyController::output()），只用 linear 和 angular
  //   dt    ：步长（秒），必须是有限的正数，否则本次调用被忽略（状态不变）
  void step(const protocol::Command& target, double dt);

  const DiffDriveState& state() const { return state_; }
  const VehicleParams& params() const { return params_; }

  // 生成协议层的遥测消息。vehicle_state 由 safety 层提供（仿真本身不知道安全状态）
  protocol::Telemetry telemetry(protocol::VehicleState vehicle_state) const;

 private:
  VehicleParams params_;
  DiffDriveState state_{};
};

}  // namespace teledrive::sim
