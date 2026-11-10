#include <gtest/gtest.h>
#include <teledrive/node/manual_input.hpp>

using teledrive::node::ManualInput;
namespace proto = teledrive::protocol;

namespace {
void press(ManualInput& in, const char* keys) {
  for (const char* p = keys; *p; ++p) in.handle_key(*p);
}
}  // namespace

TEST(ManualInput, StartsDisarmedWithZeroCommand) {
  ManualInput in;
  const auto c = in.next_command();
  EXPECT_EQ(c.flags, 0);
  EXPECT_EQ(c.linear, 0.0f);
  EXPECT_FALSE(in.armed());
}

TEST(ManualInput, SpeedKeysAreIgnoredUntilArmed) {
  ManualInput in;
  press(in, "www");
  press(in, "e");  // arm
  EXPECT_EQ(in.next_command().linear, 0.0f);  // 之前的 www 没有生效
}

TEST(ManualInput, ArmedSetsFlagAndSpeedKeysWork) {
  ManualInput in;
  press(in, "eww");
  const auto c = in.next_command();
  EXPECT_EQ(c.flags, proto::kFlagArm);
  EXPECT_NEAR(c.linear, 0.2f, 1e-6f);

  press(in, "a");
  EXPECT_NEAR(in.next_command().angular, 0.2f, 1e-6f);  // a = 左转 = 正
  press(in, "dd");
  EXPECT_NEAR(in.next_command().angular, -0.2f, 1e-6f);
}

TEST(ManualInput, SpeedIsClampedToConfiguredMaximum) {
  ManualInput in;
  press(in, "e");
  for (int i = 0; i < 100; ++i) in.handle_key('w');
  EXPECT_FLOAT_EQ(in.next_command().linear, 1.0f);
}

// 加减抵消后必须是精确的 0，否则安全层会认为"还在行驶"
TEST(ManualInput, SpeedSnapsToExactZero) {
  ManualInput in;
  press(in, "ewwwsss");
  EXPECT_EQ(in.next_command().linear, 0.0f);  // 注意是精确相等，不是 NEAR
}

TEST(ManualInput, SpaceZerosSpeedButStaysArmed) {
  ManualInput in;
  press(in, "ewww ");
  const auto c = in.next_command();
  EXPECT_EQ(c.linear, 0.0f);
  EXPECT_EQ(c.flags, proto::kFlagArm);
}

TEST(ManualInput, DisarmingClearsSpeed) {
  ManualInput in;
  press(in, "ewwe");  // arm, 加速, 再按 e 解除 arm
  const auto c = in.next_command();
  EXPECT_EQ(c.flags, 0);
  EXPECT_EQ(c.linear, 0.0f);
}

TEST(ManualInput, EStopIsRepeatedThenReturnsToDisarmed) {
  ManualInput in;
  press(in, "eww");
  in.handle_key('x');
  for (int i = 0; i < 10; ++i) {  // 默认连续 10 个周期，抗丢包
    const auto c = in.next_command();
    EXPECT_EQ(c.flags, proto::kFlagEStop) << "第 " << i << " 个";
    EXPECT_EQ(c.linear, 0.0f);
  }
  const auto after = in.next_command();
  EXPECT_EQ(after.flags, 0);   // 急停发完后是"未 arm"状态，不会自动恢复行驶
}

TEST(ManualInput, ResetSendsResetFlagOnly) {
  ManualInput in;
  in.handle_key('r');
  EXPECT_EQ(in.next_command().flags, proto::kFlagReset);
}

TEST(ManualInput, UppercaseKeysWorkAndQuitIsReported) {
  ManualInput in;
  press(in, "EW");
  EXPECT_TRUE(in.armed());
  EXPECT_FALSE(in.quit_requested());
  in.handle_key('q');
  EXPECT_TRUE(in.quit_requested());
}

// ============ 自动模式键 ============

TEST(ManualInput, AutoModeRequiresArm) {
  ManualInput in;
  in.handle_key('m');
  EXPECT_FALSE(in.auto_mode());
}

TEST(ManualInput, MTogglesAutoModeAndSetsFlag) {
  ManualInput in;
  press(in, "em");
  ASSERT_TRUE(in.auto_mode());
  const auto c = in.next_command();
  EXPECT_EQ(c.flags, proto::kFlagArm | proto::kFlagAuto);
  EXPECT_EQ(c.linear, 0.0f);

  in.handle_key('m');            // 再按一次退出
  EXPECT_FALSE(in.auto_mode());
  EXPECT_EQ(in.next_command().flags, proto::kFlagArm);
}

// 安全设计：任何手动行驶操作都立刻接管，退出自动
TEST(ManualInput, DrivingKeyTakesOverFromAuto) {
  ManualInput in;
  press(in, "emw");
  EXPECT_FALSE(in.auto_mode());
  const auto c = in.next_command();
  EXPECT_EQ(c.flags, proto::kFlagArm);
  EXPECT_NEAR(c.linear, 0.1f, 1e-6f);
}

TEST(ManualInput, SpaceAndEStopCancelAuto) {
  ManualInput a;
  press(a, "em ");
  EXPECT_FALSE(a.auto_mode());

  ManualInput b;
  press(b, "emx");
  EXPECT_FALSE(b.auto_mode());
  EXPECT_EQ(b.next_command().flags, proto::kFlagEStop);   // 急停期间只发急停标志
}

TEST(ManualInput, DisarmingCancelsAuto) {
  ManualInput in;
  press(in, "eme");
  EXPECT_FALSE(in.auto_mode());
}
