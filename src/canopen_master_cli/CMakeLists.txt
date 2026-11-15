# SPDX-License-Identifier: Apache-2.0
# canopen_master_cli: our master (the canopen backend's I/O worker) as a process for the interop
# harness on vcan. Not a library: nothing is exported.
cmake_minimum_required(VERSION 3.22)
project(canopen_master_cli LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)
find_package(runtime_core REQUIRED)
find_package(canopen_codec REQUIRED)
find_package(hal_contract REQUIRED)
find_package(canopen_master REQUIRED)
find_package(canopen_backend REQUIRED)
find_package(socketcan_io REQUIRED)

# The command line and console parsing, unit-tested without a bus.
add_library(canopen_master_cli_args STATIC src/args.cpp)
robotics_apply_flags(canopen_master_cli_args)
target_compile_features(canopen_master_cli_args PUBLIC cxx_std_20)
target_include_directories(canopen_master_cli_args PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(canopen_master_cli_args PUBLIC canopen_backend::canopen_backend
  runtime_core::runtime_core)

add_executable(canopen_master_cli src/main.cpp)
robotics_apply_flags(canopen_master_cli)
target_link_libraries(canopen_master_cli PRIVATE canopen_master_cli_args
  canopen_backend::canopen_backend hal_contract::hal_contract canopen_master::canopen_master
  socketcan_io::socketcan_io)
install(TARGETS canopen_master_cli RUNTIME DESTINATION lib/${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(canopen_master_cli_tests test/test_args.cpp)
  robotics_apply_flags(canopen_master_cli_tests)
  target_link_libraries(canopen_master_cli_tests PRIVATE canopen_master_cli_args GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(canopen_master_cli_tests)
endif()
