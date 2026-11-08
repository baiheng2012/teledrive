add_executable(common_tests
  common/vec2_test.cpp
  common/clock_test.cpp
)
target_link_libraries(common_tests PRIVATE teledrive::common GTest::gtest_main)
gtest_discover_tests(common_tests)
