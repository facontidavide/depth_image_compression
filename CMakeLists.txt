cmake_minimum_required(VERSION 3.16)
project(depth_codec VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(DEPTH_CODEC_MARCH_NATIVE "Build with -march=native (AVX2 fast paths)" ON)
option(DEPTH_CODEC_BUILD_TESTS "Build unit tests" ON)
option(DEPTH_CODEC_BUILD_BENCHMARKS "Build benchmarks" ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---- zstd: Conan/CMake config package if available, system lib otherwise ---
find_package(zstd CONFIG QUIET)
if(TARGET zstd::libzstd)
  set(DEPTH_CODEC_ZSTD zstd::libzstd)
elseif(TARGET zstd::libzstd_static)
  set(DEPTH_CODEC_ZSTD zstd::libzstd_static)
elseif(TARGET zstd::libzstd_shared)
  set(DEPTH_CODEC_ZSTD zstd::libzstd_shared)
else()
  find_path(ZSTD_INCLUDE_DIR zstd.h REQUIRED)
  find_library(ZSTD_LIBRARY NAMES zstd REQUIRED)
  add_library(depth_codec_zstd INTERFACE)
  target_include_directories(depth_codec_zstd INTERFACE ${ZSTD_INCLUDE_DIR})
  target_link_libraries(depth_codec_zstd INTERFACE ${ZSTD_LIBRARY})
  set(DEPTH_CODEC_ZSTD depth_codec_zstd)
endif()

# ---- library ---------------------------------------------------------------
add_library(depthcodec src/depth_codec.cpp)
add_library(depth_codec::depthcodec ALIAS depthcodec)
target_include_directories(depthcodec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(depthcodec PUBLIC ${DEPTH_CODEC_ZSTD})
target_compile_options(depthcodec PRIVATE -O3)
if(DEPTH_CODEC_MARCH_NATIVE)
  target_compile_options(depthcodec PRIVATE -march=native)
endif()

# ---- tests -----------------------------------------------------------------
if(DEPTH_CODEC_BUILD_TESTS)
  enable_testing()
  add_executable(test_roundtrip test/test_roundtrip.cpp)
  target_link_libraries(test_roundtrip PRIVATE depthcodec)
  add_test(NAME roundtrip COMMAND test_roundtrip)
endif()

# ---- benchmarks ------------------------------------------------------------
if(DEPTH_CODEC_BUILD_BENCHMARKS)
  # bench: ratio/throughput + bit-exact verification over a corpus directory.
  add_executable(bench benchmark/bench.cpp)
  target_link_libraries(bench PRIVATE depthcodec)

  # bench_dpred: google-benchmark micro-harness on 10 corpus frames. Uses the
  # system package if present, otherwise fetches a pinned release.
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

  add_executable(bench_dpred benchmark/bench_dpred.cpp)
  target_link_libraries(bench_dpred PRIVATE depthcodec benchmark::benchmark)
endif()

# ---- install / export ------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS depthcodec EXPORT depth_codecTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT depth_codecTargets
  NAMESPACE depth_codec::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/depth_codec)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/depth_codecConfig.cmake
  "include(CMakeFindDependencyMacro)\n"
  "find_package(zstd CONFIG QUIET)\n"
  "include(\"\${CMAKE_CURRENT_LIST_DIR}/depth_codecTargets.cmake\")\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/depth_codecConfig.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/depth_codec)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/package.xml)
  install(FILES package.xml DESTINATION share/depth_codec)
endif()
