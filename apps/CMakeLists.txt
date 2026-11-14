add_executable(vehicle_sim vehicle_sim/main.cpp)
target_link_libraries(vehicle_sim PRIVATE teledrive::node)

add_executable(operator_cli operator_cli/main.cpp)
target_link_libraries(operator_cli PRIVATE teledrive::node)

# 离线仿真工具：输出 CSV，供 tools/ 里的 Python 脚本画图
add_executable(sim_trace sim_trace.cpp)
target_link_libraries(sim_trace PRIVATE teledrive::sim teledrive::control)

add_executable(path_trace path_trace.cpp)
target_link_libraries(path_trace PRIVATE teledrive::sim teledrive::control)

# 视觉相关：只有找到 OpenCV 才构建
if(TELEDRIVE_HAVE_OPENCV)
  add_executable(render_demo render_demo.cpp)
  target_link_libraries(render_demo PRIVATE teledrive::vision)

  # 两个主程序有条件地启用视频：定义宏 TELEDRIVE_WITH_VISION，代码里用 #ifdef 包住视频部分
  foreach(app vehicle_sim operator_cli)
    target_link_libraries(${app} PRIVATE teledrive::vision)
    target_compile_definitions(${app} PRIVATE TELEDRIVE_WITH_VISION=1)
  endforeach()
endif()

# 开环对照场景：输出 CSV，供 matlab/ 里的脚本交叉验证
add_executable(sim_trace_open sim_trace_open.cpp)
target_link_libraries(sim_trace_open PRIVATE teledrive::sim)

# 性能基准：必须用 Release 构建运行
add_executable(bench bench.cpp)
target_link_libraries(bench PRIVATE teledrive::node)
