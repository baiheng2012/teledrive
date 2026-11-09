add_executable(vehicle_sim vehicle_sim/main.cpp)
target_link_libraries(vehicle_sim PRIVATE teledrive::node)

add_executable(operator_cli operator_cli/main.cpp)
target_link_libraries(operator_cli PRIVATE teledrive::node)

# 离线仿真工具：输出 CSV，供 tools/ 里的 Python 脚本画图
add_executable(sim_trace sim_trace.cpp)
target_link_libraries(sim_trace PRIVATE teledrive::sim teledrive::control)
