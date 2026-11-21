# SPDX-License-Identifier: Apache-2.0
# Independent CANopen peer for the interoperability tests (dev image only; the runtime image
# skips this package).
# Links the Lely CANopen libraries shipped by ros-jazzy-lely-core-libraries.
cmake_minimum_required(VERSION 3.22)
project(lely_peer LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)
find_package(PkgConfig REQUIRED)
# Imported targets: their include directories are SYSTEM, so Lely's headers stay out of -Werror.
pkg_check_modules(LELY REQUIRED IMPORTED_TARGET liblely-coapp liblely-io2 liblely-ev)

# Lely-free helpers, unit-tested without a bus.
add_library(lely_peer_args STATIC src/args.cpp)
robotics_apply_flags(lely_peer_args)
target_compile_features(lely_peer_args PUBLIC cxx_std_20)
target_include_directories(lely_peer_args PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

foreach(exe lely_peer probe_slave)
  if(exe STREQUAL "lely_peer")
    add_executable(${exe} src/main.cpp)
  else()
    add_executable(${exe} src/${exe}.cpp)
  endif()
  robotics_apply_flags(${exe})
  target_link_libraries(${exe} PRIVATE lely_peer_args PkgConfig::LELY)
  set_target_properties(${exe} PROPERTIES BUILD_RPATH "${LELY_LIBRARY_DIRS}"
                                          INSTALL_RPATH "${LELY_LIBRARY_DIRS}")
  install(TARGETS ${exe} RUNTIME DESTINATION lib/${PROJECT_NAME})
endforeach()

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(lely_peer_tests test/test_args.cpp)
  robotics_apply_flags(lely_peer_tests)
  target_link_libraries(lely_peer_tests PRIVATE lely_peer_args GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(lely_peer_tests)
endif()
