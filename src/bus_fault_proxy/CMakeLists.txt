# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(bus_fault_proxy LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)
find_package(runtime_core REQUIRED)
find_package(canopen_codec REQUIRED)

file(GLOB BUS_FAULT_PROXY_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(bus_fault_proxy ${BUS_FAULT_PROXY_SOURCES})
add_library(bus_fault_proxy::bus_fault_proxy ALIAS bus_fault_proxy)
robotics_apply_flags(bus_fault_proxy)
# The public headers need C++20; an outside consumer of the exported target gets it from here.
target_compile_features(bus_fault_proxy PUBLIC cxx_std_20)
target_include_directories(bus_fault_proxy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bus_fault_proxy PUBLIC
  runtime_core::runtime_core canopen_codec::canopen_codec)

# Consumable package: find_package(bus_fault_proxy REQUIRED) -> bus_fault_proxy::bus_fault_proxy.
install(TARGETS bus_fault_proxy EXPORT bus_fault_proxyTargets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT bus_fault_proxyTargets NAMESPACE bus_fault_proxy:: DESTINATION share/bus_fault_proxy/cmake)
install(FILES cmake/bus_fault_proxyConfig.cmake DESTINATION share/bus_fault_proxy/cmake)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  file(GLOB BUS_FAULT_PROXY_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  add_executable(bus_fault_proxy_tests ${BUS_FAULT_PROXY_TESTS})
  robotics_apply_flags(bus_fault_proxy_tests)
  target_link_libraries(bus_fault_proxy_tests PRIVATE bus_fault_proxy GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(bus_fault_proxy_tests)
endif()
