#include <gtest/gtest.h>
#include <teledrive/safety/watchdog.hpp>

using namespace std::chrono_literals;
using teledrive::FakeClock;
using teledrive::safety::Watchdog;

TEST(Watchdog, NeverExpiresBeforeFirstFeed) {
  FakeClock clock;
  Watchdog wd(clock, 200ms);
  clock.advance(10s);  // 就算过了很久，没启动就不报超时
  EXPECT_FALSE(wd.running());
  EXPECT_FALSE(wd.expired());
}

// 边界：恰好等于 timeout 不算超时，多 1ms 才算。把语义钉死。
TEST(Watchdog, ExpiresOnlyAfterTimeoutElapsed) {
  FakeClock clock;
  Watchdog wd(clock, 200ms);
  wd.feed();

  clock.advance(200ms);
  EXPECT_FALSE(wd.expired());

  clock.advance(1ms);
  EXPECT_TRUE(wd.expired());
}

TEST(Watchdog, FeedingRestartsTheCountdown) {
  FakeClock clock;
  Watchdog wd(clock, 200ms);
  wd.feed();
  clock.advance(150ms);
  wd.feed();            // 喂狗，重新计时
  clock.advance(150ms); // 距离上次喂狗只有 150ms
  EXPECT_FALSE(wd.expired());
}

TEST(Watchdog, StopDisablesExpiry) {
  FakeClock clock;
  Watchdog wd(clock, 200ms);
  wd.feed();
  clock.advance(1s);
  ASSERT_TRUE(wd.expired());

  wd.stop();
  EXPECT_FALSE(wd.expired());
  EXPECT_FALSE(wd.running());
}

TEST(Watchdog, CanBeRestartedAfterStop) {
  FakeClock clock;
  Watchdog wd(clock, 200ms);
  wd.feed();
  wd.stop();
  clock.advance(1s);
  wd.feed();  // 重新启动，从现在开始计时
  EXPECT_TRUE(wd.running());
  EXPECT_FALSE(wd.expired());
}
