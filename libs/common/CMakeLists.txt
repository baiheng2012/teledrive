add_library(common INTERFACE)
add_library(teledrive::common ALIAS common)
target_include_directories(common INTERFACE include)
