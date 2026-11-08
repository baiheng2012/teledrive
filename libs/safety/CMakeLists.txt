add_library(safety STATIC
  src/limits.cpp
  src/safety_controller.cpp
)
add_library(teledrive::safety ALIAS safety)

target_include_directories(safety PUBLIC include)

# safety 使用 Command / VehicleState（来自 protocol）和 IClock（来自 common）。
# 注意：它不依赖 transport，安全逻辑与"字节怎么传"完全无关。
target_link_libraries(safety PUBLIC teledrive::common teledrive::protocol)
