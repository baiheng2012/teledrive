#pragma once
// 键盘输入到指令的映射。独立成类的原因：键盘读取（termios）没法测试，
// 但"按了什么键 → 产生什么指令"这套规则是可以、也应该被测试的。
//
// 按键：
//   e       arm / disarm 切换
//   w / s   线速度 +/- 一档      （仅在 arm 后有效）
//   a / d   角速度 +/- 一档      （a = 左转 = 正）
//   空格     速度归零
//   x       急停
//   r       复位故障
//   q       退出
//
//   m       自动模式开关（需先 arm；任何行驶键/空格/急停都会立刻退出自动）
// 已知局限：键盘无法检测"按键松开"，所以做不到真正的死手开关，
// 这里用"arm 开关"代替。真实手持遥控器靠硬件开关实现。

#include <cstdint>

#include "teledrive/protocol/messages.hpp"

namespace teledrive::node {

// 放在命名空间层级而不是类内部：带默认成员初值的嵌套类型
// 用作默认实参时，部分编译器会报"初始化器在类结束前需要"的错误。
struct ManualInputConfig {
  float linear_step{0.1f};     // 每按一次 w/s 改变多少 m/s
  float angular_step{0.2f};    // 每按一次 a/d 改变多少 rad/s
  float max_linear{1.0f};
  float max_angular{1.5f};
  int flag_repeat{10};         // 急停/复位标志连续发送多少个周期（防 UDP 丢包）
};

class ManualInput {
 public:
  explicit ManualInput(ManualInputConfig config = {}) : config_(config) {}

  void handle_key(char key);

  // 每个控制周期调用一次，生成要发送的指令。
  // 注意它会修改内部计数（急停/复位标志的剩余发送次数）。
  protocol::Command next_command();

  bool armed() const { return armed_; }
  bool quit_requested() const { return quit_; }
  bool auto_mode() const {return auto_;}
 private:
  void nudge(float& value, float delta, float limit);
  void stop_motion() { linear_ = 0.0f; angular_ = 0.0f; }

  ManualInputConfig config_;
  bool armed_{false};
  bool auto_{false};
  bool quit_{false};
  float linear_{0.0f};
  float angular_{0.0f};
  int estop_left_{0};   // 还要连续发送几次急停标志
  int reset_left_{0};
};

}  // namespace teledrive::node
