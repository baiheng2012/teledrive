add_library(node STATIC
  src/vehicle_node.cpp
  src/operator_session.cpp
  src/manual_input.cpp
)
add_library(teledrive::node ALIAS node)

target_include_directories(node PUBLIC include)

# node 是"组装层"：把前面所有库组合起来，所以依赖它们。
# 反过来，前面的库互相保持独立，不会反向依赖 node。
target_link_libraries(node PUBLIC
  teledrive::common teledrive::protocol teledrive::transport
  teledrive::safety teledrive::sim)
target_link_libraries(node PUBLIC
  teledrive::common teledrive::protocol teledrive::transport
  teledrive::safety teledrive::sim teledrive::control)
