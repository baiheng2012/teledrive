cmake_minimum_required(VERSION 3.24)
project(teledrive VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)

option(TELEDRIVE_BUILD_TESTS "Build unit tests" ON)
option(TELEDRIVE_WITH_OPENCV "Build the OpenCV-based vision module" ON)
# CI 和 Docker 打开它：缺 OpenCV 就让配置直接失败，而不是悄悄跳过 vision 测试
option(TELEDRIVE_REQUIRE_OPENCV "Fail configuration if OpenCV is missing" OFF)

set(TELEDRIVE_HAVE_OPENCV OFF)
if(TELEDRIVE_WITH_OPENCV)
  # QUIET：找不到时不报红色错误；COMPONENTS：只要用到的三个模块
  find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs)
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
