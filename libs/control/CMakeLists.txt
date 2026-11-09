add_library(control STATIC
  src/pid.cpp
  src/heading_hold.cpp
  src/speed_limiter.cpp
)
add_library(teledrive::control ALIAS control)
target_include_directories(control PUBLIC include)
target_link_libraries(control PUBLIC teledrive::common teledrive::protocol teledrive::sim)
