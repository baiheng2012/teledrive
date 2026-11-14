add_library(can STATIC
  src/can_mapping.cpp
  src/can_transport.cpp
  src/mock_can_bus.cpp
  src/socketcan_bus.cpp
)
add_library(teledrive::can ALIAS can)
target_include_directories(can PUBLIC include)
# can 用到 ITransport（transport）和协议常量（protocol）
target_link_libraries(can PUBLIC teledrive::common teledrive::protocol teledrive::transport)
