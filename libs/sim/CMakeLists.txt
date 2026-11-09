add_library(sim STATIC
  src/diff_drive.cpp
  src/lidar.cpp
)
add_library(teledrive::sim ALIAS sim)

target_include_directories(sim PUBLIC include)

# sim 用到 Vec2（common）和 Command/Telemetry（protocol）。
# 刻意不依赖 safety 和 transport：车辆模型只关心"物理"，
# 谁来决定它能不能动，是组装层（应用程序）的事。
target_link_libraries(sim PUBLIC teledrive::common teledrive::protocol)
