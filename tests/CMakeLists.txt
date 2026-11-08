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
