# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(hal_contract LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)
find_package(runtime_core REQUIRED)

file(GLOB HAL_CONTRACT_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(hal_contract ${HAL_CONTRACT_SOURCES})
add_library(hal_contract::hal_contract ALIAS hal_contract)
robotics_apply_flags(hal_contract)
# The public headers need C++20; an outside consumer of the exported target gets it from here.
target_compile_features(hal_contract PUBLIC cxx_std_20)
target_include_directories(hal_contract PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hal_contract PUBLIC runtime_core::runtime_core)

# Consumable package: find_package(hal_contract REQUIRED) -> hal_contract::hal_contract.
# include/hal_contract/testing/ is installed too: every backend runs the same contract suite.
install(TARGETS hal_contract EXPORT hal_contractTargets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT hal_contractTargets NAMESPACE hal_contract:: DESTINATION share/hal_contract/cmake)
install(FILES cmake/hal_contractConfig.cmake DESTINATION share/hal_contract/cmake)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  file(GLOB HAL_CONTRACT_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  add_executable(hal_contract_tests ${HAL_CONTRACT_TESTS})
  robotics_apply_flags(hal_contract_tests)
  target_link_libraries(hal_contract_tests PRIVATE hal_contract GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(hal_contract_tests)
endif()
