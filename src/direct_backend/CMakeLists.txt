# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(direct_backend LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)
find_package(runtime_core REQUIRED)
find_package(hal_contract REQUIRED)
find_package(drive_core REQUIRED)

file(GLOB DIRECT_BACKEND_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(direct_backend ${DIRECT_BACKEND_SOURCES})
add_library(direct_backend::direct_backend ALIAS direct_backend)
robotics_apply_flags(direct_backend)
# The public headers need C++20; an outside consumer of the exported target gets it from here.
target_compile_features(direct_backend PUBLIC cxx_std_20)
target_include_directories(direct_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(direct_backend PUBLIC
  runtime_core::runtime_core hal_contract::hal_contract drive_core::drive_core)

# Consumable package: find_package(direct_backend REQUIRED) -> direct_backend::direct_backend.
install(TARGETS direct_backend EXPORT direct_backendTargets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT direct_backendTargets NAMESPACE direct_backend:: DESTINATION share/direct_backend/cmake)
install(FILES cmake/direct_backendConfig.cmake DESTINATION share/direct_backend/cmake)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  file(GLOB DIRECT_BACKEND_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  add_executable(direct_backend_tests ${DIRECT_BACKEND_TESTS})
  robotics_apply_flags(direct_backend_tests)
  target_link_libraries(direct_backend_tests PRIVATE direct_backend GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(direct_backend_tests)
endif()
