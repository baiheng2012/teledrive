#include <gtest/gtest.h>
#include <teledrive/common/clock.hpp>

using namespace std::chrono_literals;
using teledrive::FakeClock;

TEST(FakeClock, DoesNotMoveByItself) {
  FakeClock c;
  EXPECT_EQ(c.now(), c.now());
}

TEST(FakeClock, AdvancesExactly) {
  FakeClock c;
  auto t0 = c.now();
  c.advance(200ms);
  EXPECT_EQ(c.now() - t0, 200ms);
}
