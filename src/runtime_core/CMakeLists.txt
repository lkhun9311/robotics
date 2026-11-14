# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(runtime_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(RUNTIME_CORE_SANITIZE "" CACHE STRING "Comma-separated -fsanitize list, empty to disable")
option(RUNTIME_CORE_COVERAGE "Instrument for gcov coverage" OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
if(RUNTIME_CORE_SANITIZE)
  add_compile_options(-fsanitize=${RUNTIME_CORE_SANITIZE} -fno-omit-frame-pointer)
  add_link_options(-fsanitize=${RUNTIME_CORE_SANITIZE})
endif()
if(RUNTIME_CORE_COVERAGE)
  add_compile_options(--coverage)
  add_link_options(--coverage)
endif()

file(GLOB RUNTIME_CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(runtime_core ${RUNTIME_CORE_SOURCES})
target_include_directories(runtime_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# A CONFIGURE_DEPENDS glob (not if(EXISTS)) so adding the harness later re-runs configure.
file(GLOB RUNTIME_CORE_HARNESS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/harness/logic_sim.cpp)
if(RUNTIME_CORE_HARNESS)
  add_executable(logic_sim ${RUNTIME_CORE_HARNESS})
  target_link_libraries(logic_sim PRIVATE runtime_core)
  install(TARGETS logic_sim RUNTIME DESTINATION lib/runtime_core)
endif()

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  file(GLOB RUNTIME_CORE_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  add_executable(runtime_core_tests ${RUNTIME_CORE_TESTS})
  target_link_libraries(runtime_core_tests PRIVATE runtime_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(runtime_core_tests)
endif()
