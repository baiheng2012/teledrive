add_library(control STATIC
  src/pid.cpp
  src/heading_hold.cpp
  src/speed_limiter.cpp
  src/pure_pursuit.cpp
  src/generated_pid.cpp
  generated/pid_ctrl.cpp
)
add_library(teledrive::control ALIAS control)
target_include_directories(control PUBLIC include)
# 生成代码的头文件只给 control 自己用；SYSTEM 避免我们的严格警告去挑生成代码的毛病
target_include_directories(control SYSTEM PRIVATE generated)
target_link_libraries(control PUBLIC teledrive::common teledrive::protocol teledrive::sim)
