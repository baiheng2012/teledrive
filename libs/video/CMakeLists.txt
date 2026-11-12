add_library(video STATIC src/video_packet.cpp)
add_library(teledrive::video ALIAS video)
target_include_directories(video PUBLIC include)
# 复用 protocol 里的 byte_io 和 crc16
target_link_libraries(video PUBLIC teledrive::protocol)
