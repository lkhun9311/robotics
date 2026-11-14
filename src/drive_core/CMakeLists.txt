# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(drive_core LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)
find_package(runtime_core REQUIRED)
find_package(canopen_codec REQUIRED)

file(GLOB DRIVE_CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(drive_core ${DRIVE_CORE_SOURCES})
add_library(drive_core::drive_core ALIAS drive_core)
robotics_apply_flags(drive_core)
target_include_directories(drive_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(drive_core PUBLIC runtime_core::runtime_core canopen_codec::canopen_codec)

# Consumable package: find_package(drive_core REQUIRED) -> drive_core::drive_core.
install(TARGETS drive_core EXPORT drive_coreTargets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT drive_coreTargets NAMESPACE drive_core:: DESTINATION share/drive_core/cmake)
install(FILES cmake/drive_coreConfig.cmake DESTINATION share/drive_core/cmake)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  file(GLOB DRIVE_CORE_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  add_executable(drive_core_tests ${DRIVE_CORE_TESTS})
  robotics_apply_flags(drive_core_tests)
  target_link_libraries(drive_core_tests PRIVATE drive_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(drive_core_tests)
endif()
