# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(runtime_core LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)

file(GLOB RUNTIME_CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(runtime_core ${RUNTIME_CORE_SOURCES})
add_library(runtime_core::runtime_core ALIAS runtime_core)
robotics_apply_flags(runtime_core)
target_include_directories(runtime_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Consumable package: find_package(runtime_core REQUIRED) -> runtime_core::runtime_core.
install(TARGETS runtime_core EXPORT runtime_coreTargets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT runtime_coreTargets NAMESPACE runtime_core:: DESTINATION share/runtime_core/cmake)
install(FILES cmake/runtime_coreConfig.cmake DESTINATION share/runtime_core/cmake)

# A CONFIGURE_DEPENDS glob (not if(EXISTS)) so adding the harness later re-runs configure.
file(GLOB RUNTIME_CORE_HARNESS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/harness/logic_sim.cpp)
if(RUNTIME_CORE_HARNESS)
  add_executable(logic_sim ${RUNTIME_CORE_HARNESS})
  robotics_apply_flags(logic_sim)
  target_link_libraries(logic_sim PRIVATE runtime_core)
  install(TARGETS logic_sim RUNTIME DESTINATION lib/runtime_core)
endif()

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  file(GLOB RUNTIME_CORE_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  add_executable(runtime_core_tests ${RUNTIME_CORE_TESTS})
  robotics_apply_flags(runtime_core_tests)
  target_link_libraries(runtime_core_tests PRIVATE runtime_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(runtime_core_tests)
endif()
