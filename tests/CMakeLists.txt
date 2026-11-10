add_executable(common_tests
  common/vec2_test.cpp
  common/clock_test.cpp
)
target_link_libraries(common_tests PRIVATE teledrive::common GTest::gtest_main)
gtest_discover_tests(common_tests)
add_executable(protocol_tests
  protocol/crc16_test.cpp
  protocol/messages_test.cpp
)
target_link_libraries(protocol_tests PRIVATE teledrive::protocol GTest::gtest_main)
gtest_discover_tests(protocol_tests)
add_executable(transport_tests
  transport/endpoint_test.cpp
  transport/mock_transport_test.cpp
  transport/udp_transport_test.cpp
)
# 测试里要演示"协议 + 传输"配合，所以同时链接 protocol
target_link_libraries(transport_tests PRIVATE
  teledrive::transport teledrive::protocol GTest::gtest_main)
gtest_discover_tests(transport_tests)

add_executable(safety_tests
  safety/limits_test.cpp
  safety/watchdog_test.cpp
  safety/safety_controller_test.cpp
)
target_link_libraries(safety_tests PRIVATE teledrive::safety GTest::gtest_main)
gtest_discover_tests(safety_tests)

add_executable(sim_tests
  sim/diff_drive_test.cpp
  sim/closed_loop_test.cpp
  sim/lidar_test.cpp
)
# 闭环测试同时用到 sim 和 safety
target_link_libraries(sim_tests PRIVATE teledrive::sim teledrive::safety GTest::gtest_main)
gtest_discover_tests(sim_tests)

add_executable(node_tests
  node/end_to_end_test.cpp
  node/manual_input_test.cpp
  node/obstacle_assist_test.cpp
  node/auto_mode_test.cpp

)
target_link_libraries(node_tests PRIVATE teledrive::node GTest::gtest_main)
gtest_discover_tests(node_tests)

add_executable(control_tests
  control/pid_test.cpp
  control/heading_hold_test.cpp
  control/speed_limiter_test.cpp
  control/obstacle_assist_closed_loop_test.cpp
  control/pure_pursuit_test.cpp
)
target_link_libraries(control_tests PRIVATE teledrive::control teledrive::sim GTest::gtest_main)
gtest_discover_tests(control_tests)
