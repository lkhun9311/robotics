# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(canopen_master LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)
find_package(runtime_core REQUIRED)
find_package(canopen_codec REQUIRED)

file(GLOB CANOPEN_MASTER_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(canopen_master ${CANOPEN_MASTER_SOURCES})
add_library(canopen_master::canopen_master ALIAS canopen_master)
robotics_apply_flags(canopen_master)
# The public headers need C++20; an outside consumer of the exported target gets it from here.
target_compile_features(canopen_master PUBLIC cxx_std_20)
target_include_directories(canopen_master PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(canopen_master PUBLIC
  runtime_core::runtime_core canopen_codec::canopen_codec)

# Consumable package: find_package(canopen_master REQUIRED) -> canopen_master::canopen_master.
install(TARGETS canopen_master EXPORT canopen_masterTargets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT canopen_masterTargets NAMESPACE canopen_master:: DESTINATION share/canopen_master/cmake)
install(FILES cmake/canopen_masterConfig.cmake DESTINATION share/canopen_master/cmake)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  file(GLOB CANOPEN_MASTER_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  add_executable(canopen_master_tests ${CANOPEN_MASTER_TESTS})
  robotics_apply_flags(canopen_master_tests)
  target_link_libraries(canopen_master_tests PRIVATE canopen_master GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(canopen_master_tests)
endif()
