add_library(vision STATIC
  src/scene_renderer.cpp
  src/jpeg_codec.cpp
  src/video_sender.cpp
  src/video_receiver.cpp
)
add_library(teledrive::vision ALIAS vision)
target_include_directories(vision PUBLIC include)

# SYSTEM：OpenCV 头文件当系统头文件，我们的 -Wpedantic 不去挑它们的毛病
target_include_directories(vision SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})

# 新增 transport：发送器和接收器要用 ITransport
target_link_libraries(vision PUBLIC
  teledrive::common teledrive::sim teledrive::video teledrive::transport ${OpenCV_LIBS})
