add_library(transport STATIC
  src/endpoint.cpp
  src/udp_transport.cpp
  src/mock_transport.cpp
)
add_library(teledrive::transport ALIAS transport)

target_include_directories(transport PUBLIC include)

# transport 本身只搬运字节，不需要知道协议内容，所以不依赖 protocol。
# 这是刻意的分层：protocol 和 transport 互相独立，只在应用层组合。
target_link_libraries(transport PUBLIC teledrive::common)
