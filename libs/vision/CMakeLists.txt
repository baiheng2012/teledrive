add_library(vision STATIC
  src/scene_renderer.cpp
  src/jpeg_codec.cpp
)
add_library(teledrive::vision ALIAS vision)
target_include_directories(vision PUBLIC include)

# SYSTEM：把 OpenCV 头文件当成"系统头文件"，我们的 -Wpedantic 不会去挑它们的毛病
target_include_directories(vision SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})

# OpenCV_LIBS 由 find_package(OpenCV) 提供（opencv_core 等）
target_link_libraries(vision PUBLIC
  teledrive::common teledrive::sim teledrive::video ${OpenCV_LIBS})
