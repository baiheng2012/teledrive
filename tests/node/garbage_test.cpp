// 往车辆节点灌入大量随机数据报：车必须纹丝不动，所有数据都被计为坏包。
#include <gtest/gtest.h>
#include <teledrive/node/vehicle_node.hpp>
#include <teledrive/transport/mock_transport.hpp>

#include <array>
#include <random>

using namespace std::chrono_literals;

TEST(NodeFuzz, GarbageDatagramsNeverMoveTheVehicle) {
  teledrive::FakeClock clock;
  const teledrive::transport::Endpoint vehicle_ep{0x7F000001u, 9001};
  const teledrive::transport::Endpoint attacker{0x0A000009u, 4444};
  teledrive::transport::MockTransport transport(vehicle_ep);
  teledrive::node::VehicleNode node(transport, clock);

  std::mt19937 rng(2024);
  std::array<std::uint8_t, 300> buf{};
  constexpr int kTotal = 20000;

  for (int i = 0; i < kTotal; ++i) {
    const std::size_t len = rng() % (buf.size() + 1);   // 长度 0 也包含
    for (std::size_t j = 0; j < len; ++j) buf[j] = static_cast<std::uint8_t>(rng());
    transport.inject(attacker, buf.data(), len);
    // 每周期最多处理 64 个包，所以每注入 50 个就 tick 一次
    if (i % 50 == 49) {
      node.tick();
      clock.advance(20ms);
    }
  }
  node.tick();

  EXPECT_EQ(node.stats().rx_ok, 0u);
  EXPECT_EQ(node.stats().rx_bad, static_cast<std::uint64_t>(kTotal));
  EXPECT_EQ(node.safety().state(), teledrive::protocol::VehicleState::Idle);
  EXPECT_DOUBLE_EQ(node.model().state().pose.position.x, 0.0);
}
