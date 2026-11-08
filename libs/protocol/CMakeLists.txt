# protocol 有 .cpp 文件，所以是普通静态库（不再是 INTERFACE）
add_library(protocol STATIC
  src/crc16.cpp
  src/messages.cpp
)
add_library(teledrive::protocol ALIAS protocol)

# PUBLIC：使用 protocol 的目标也能看到这个头文件目录
target_include_directories(protocol PUBLIC include)

# protocol 依赖 common（以后会用到 Vec2 等），PUBLIC 表示依赖会传递
target_link_libraries(protocol PUBLIC teledrive::common)
