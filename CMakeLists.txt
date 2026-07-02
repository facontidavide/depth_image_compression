cmake_minimum_required(VERSION 3.16)
project(depth_codec CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")

find_path(ZSTD_INCLUDE_DIR zstd.h)
find_library(ZSTD_LIBRARY NAMES zstd)
if(NOT ZSTD_INCLUDE_DIR OR NOT ZSTD_LIBRARY)
  message(FATAL_ERROR "libzstd not found")
endif()

add_library(depthcodec depth_codec.cpp)
target_include_directories(depthcodec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${ZSTD_INCLUDE_DIR})
target_link_libraries(depthcodec PUBLIC ${ZSTD_LIBRARY})

add_executable(bench bench.cpp)
target_link_libraries(bench PRIVATE depthcodec)

# Google Benchmark micro-harness (bench_dpred): use the system package if
# present, otherwise fetch a pinned release at configure time.
find_package(benchmark QUIET)
if(NOT benchmark_FOUND)
  include(FetchContent)
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG v1.9.1
  )
  FetchContent_MakeAvailable(benchmark)
endif()

add_executable(bench_dpred bench_dpred.cpp)
target_link_libraries(bench_dpred PRIVATE depthcodec benchmark::benchmark)
