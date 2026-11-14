# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(socketcan_io LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)
find_package(runtime_core REQUIRED)
find_package(canopen_codec REQUIRED)

file(GLOB SOCKETCAN_IO_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(socketcan_io ${SOCKETCAN_IO_SOURCES})
add_library(socketcan_io::socketcan_io ALIAS socketcan_io)
robotics_apply_flags(socketcan_io)
# The public headers need C++20; an outside consumer of the exported target gets it from here.
target_compile_features(socketcan_io PUBLIC cxx_std_20)
target_include_directories(socketcan_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(socketcan_io PUBLIC runtime_core::runtime_core canopen_codec::canopen_codec)

# Consumable package: find_package(socketcan_io REQUIRED) -> socketcan_io::socketcan_io.
install(TARGETS socketcan_io EXPORT socketcan_ioTargets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT socketcan_ioTargets NAMESPACE socketcan_io:: DESTINATION share/socketcan_io/cmake)
install(FILES cmake/socketcan_ioConfig.cmake DESTINATION share/socketcan_io/cmake)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  file(GLOB SOCKETCAN_IO_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  add_executable(socketcan_io_tests ${SOCKETCAN_IO_TESTS})
  robotics_apply_flags(socketcan_io_tests)
  target_link_libraries(socketcan_io_tests PRIVATE socketcan_io GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(socketcan_io_tests)
endif()
