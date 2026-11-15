# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(canopen_backend LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)
find_package(runtime_core REQUIRED)
find_package(canopen_codec REQUIRED)
find_package(hal_contract REQUIRED)
find_package(canopen_master REQUIRED)

file(GLOB CANOPEN_BACKEND_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(canopen_backend ${CANOPEN_BACKEND_SOURCES})
add_library(canopen_backend::canopen_backend ALIAS canopen_backend)
robotics_apply_flags(canopen_backend)
# The public headers need C++20; an outside consumer of the exported target gets it from here.
target_compile_features(canopen_backend PUBLIC cxx_std_20)
target_include_directories(canopen_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(canopen_backend PUBLIC
  runtime_core::runtime_core canopen_codec::canopen_codec hal_contract::hal_contract
  canopen_master::canopen_master)

# Consumable package: find_package(canopen_backend REQUIRED) -> canopen_backend::canopen_backend.
install(TARGETS canopen_backend EXPORT canopen_backendTargets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT canopen_backendTargets NAMESPACE canopen_backend:: DESTINATION share/canopen_backend/cmake)
install(FILES cmake/canopen_backendConfig.cmake DESTINATION share/canopen_backend/cmake)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  # The tests run the backend against six in-memory drive_core::Drive (the shared DriveBus).
  find_package(drive_core REQUIRED)
  file(GLOB CANOPEN_BACKEND_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  add_executable(canopen_backend_tests ${CANOPEN_BACKEND_TESTS})
  robotics_apply_flags(canopen_backend_tests)
  target_link_libraries(canopen_backend_tests PRIVATE canopen_backend drive_core::drive_core
    GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(canopen_backend_tests)
endif()
