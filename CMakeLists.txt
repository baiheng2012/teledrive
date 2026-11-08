cmake_minimum_required(VERSION 3.24)
project(teledrive VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)

option(TELEDRIVE_BUILD_TESTS "Build unit tests" ON)

add_subdirectory(libs/common)
add_subdirectory(libs/protocol)
add_subdirectory(libs/transport)
add_subdirectory(libs/safety)
add_subdirectory(libs/sim)



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
