cmake_minimum_required(VERSION 3.24)
project(teledrive VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)

# ---- 质量工具开关（默认关闭，不影响日常构建）----
option(TELEDRIVE_SANITIZE "Build with AddressSanitizer and UBSan" OFF)
option(TELEDRIVE_COVERAGE "Build with gcov coverage instrumentation" OFF)

# 两者不能同时开：sanitizer 改变了代码生成，覆盖率数据会失真
if(TELEDRIVE_SANITIZE AND TELEDRIVE_COVERAGE)
  message(FATAL_ERROR "TELEDRIVE_SANITIZE and TELEDRIVE_COVERAGE are mutually exclusive")
endif()

if(TELEDRIVE_SANITIZE)
  # address：内存错误；undefined：未定义行为
  # -fno-sanitize-recover=undefined：UBSan 默认只打印警告然后继续运行，测试照样显示通过。
  #   加上它，发现未定义行为就直接终止程序，测试才会真的失败
  # -fno-omit-frame-pointer：让报错时的调用栈更完整
  add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=undefined
                      -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

if(TELEDRIVE_COVERAGE)
  # --coverage = -fprofile-arcs -ftest-coverage；-O0：关闭优化，行号才能精确对应
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()





option(TELEDRIVE_BUILD_TESTS "Build unit tests" ON)
option(TELEDRIVE_WITH_OPENCV "Build the OpenCV-based vision module" ON)
# CI 和 Docker 打开它：缺 OpenCV 就让配置直接失败，而不是悄悄跳过 vision 测试
option(TELEDRIVE_REQUIRE_OPENCV "Fail configuration if OpenCV is missing" OFF)

set(TELEDRIVE_HAVE_OPENCV OFF)
if(TELEDRIVE_WITH_OPENCV)
  # QUIET：找不到时不报红色错误；COMPONENTS：只要用到的三个模块
  find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs highgui)
  if(OpenCV_FOUND)
    set(TELEDRIVE_HAVE_OPENCV ON)
    message(STATUS "OpenCV ${OpenCV_VERSION} found: vision module enabled")
  elseif(TELEDRIVE_REQUIRE_OPENCV)
    message(FATAL_ERROR "OpenCV is required but was not found")
  else()
    message(WARNING "OpenCV not found: vision module disabled (sudo apt install libopencv-dev)")
  endif()
endif()
add_subdirectory(libs/common)
add_subdirectory(libs/protocol)
add_subdirectory(libs/transport)
add_subdirectory(libs/can)
add_subdirectory(libs/safety)
add_subdirectory(libs/sim)
add_subdirectory(libs/control)
add_subdirectory(libs/video)
if(TELEDRIVE_HAVE_OPENCV)
  add_subdirectory(libs/vision)
endif()
add_subdirectory(libs/node)
add_subdirectory(apps)


if(TELEDRIVE_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE

  )
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  include(GoogleTest)
  add_subdirectory(tests)
endif()
