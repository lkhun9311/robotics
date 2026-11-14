# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(vdrive_bank LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)
find_package(runtime_core REQUIRED)
find_package(canopen_codec REQUIRED)
find_package(drive_core REQUIRED)
find_package(socketcan_io REQUIRED)
find_package(Threads REQUIRED)

# The library is every source but the process entry point (main.cpp).
file(GLOB VDRIVE_BANK_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM VDRIVE_BANK_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
add_library(vdrive_bank ${VDRIVE_BANK_SOURCES})
add_library(vdrive_bank::vdrive_bank ALIAS vdrive_bank)
robotics_apply_flags(vdrive_bank)
# The public headers need C++20; an outside consumer of the exported target gets it from here.
target_compile_features(vdrive_bank PUBLIC cxx_std_20)
target_include_directories(vdrive_bank PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vdrive_bank PUBLIC
  runtime_core::runtime_core canopen_codec::canopen_codec drive_core::drive_core
  socketcan_io::socketcan_io Threads::Threads)

# The process: vdrive_bank --if IF --trace PATH [--inject PATH].
add_executable(vdrive_bank_exe src/main.cpp)
set_target_properties(vdrive_bank_exe PROPERTIES OUTPUT_NAME vdrive_bank)
robotics_apply_flags(vdrive_bank_exe)
target_link_libraries(vdrive_bank_exe PRIVATE vdrive_bank)
install(TARGETS vdrive_bank_exe RUNTIME DESTINATION lib/${PROJECT_NAME})

# Consumable package: find_package(vdrive_bank REQUIRED) -> vdrive_bank::vdrive_bank.
install(TARGETS vdrive_bank EXPORT vdrive_bankTargets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT vdrive_bankTargets NAMESPACE vdrive_bank:: DESTINATION share/vdrive_bank/cmake)
install(FILES cmake/vdrive_bankConfig.cmake DESTINATION share/vdrive_bank/cmake)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  file(GLOB VDRIVE_BANK_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  add_executable(vdrive_bank_tests ${VDRIVE_BANK_TESTS})
  robotics_apply_flags(vdrive_bank_tests)
  target_link_libraries(vdrive_bank_tests PRIVATE vdrive_bank GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(vdrive_bank_tests)
endif()
