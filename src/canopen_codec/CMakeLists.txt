# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(canopen_codec LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# The object dictionary header is generated from the EDS on every EDS or generator change.
# The generator exits non-zero on an EDS it rejects and then writes nothing, so the build
# fails; an older header stays older than the EDS and is regenerated on the next build.
set(CANOPEN_CODEC_EDS ${CMAKE_CURRENT_SOURCE_DIR}/../../config/od/vdrive_arm.eds)
set(CANOPEN_CODEC_GEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/od)
set(CANOPEN_CODEC_GEN_INCLUDE ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(CANOPEN_CODEC_OD_HEADER ${CANOPEN_CODEC_GEN_INCLUDE}/canopen_codec/od.hpp)
add_custom_command(
  OUTPUT ${CANOPEN_CODEC_OD_HEADER}
  COMMAND ${Python3_EXECUTABLE} ${CANOPEN_CODEC_GEN_DIR}/gen_header.py
          ${CANOPEN_CODEC_EDS} ${CANOPEN_CODEC_OD_HEADER}
  DEPENDS ${CANOPEN_CODEC_EDS} ${CANOPEN_CODEC_GEN_DIR}/gen_header.py ${CANOPEN_CODEC_GEN_DIR}/od.py
  COMMENT "Generating canopen_codec/od.hpp from vdrive_arm.eds"
  VERBATIM)
add_custom_target(canopen_codec_od_header DEPENDS ${CANOPEN_CODEC_OD_HEADER})

file(GLOB CANOPEN_CODEC_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(canopen_codec ${CANOPEN_CODEC_SOURCES} ${CANOPEN_CODEC_OD_HEADER})
add_library(canopen_codec::canopen_codec ALIAS canopen_codec)
add_dependencies(canopen_codec canopen_codec_od_header)
robotics_apply_flags(canopen_codec)
target_include_directories(canopen_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CANOPEN_CODEC_GEN_INCLUDE}>
  $<INSTALL_INTERFACE:include>)

# Consumable package: find_package(canopen_codec REQUIRED) -> canopen_codec::canopen_codec.
install(TARGETS canopen_codec EXPORT canopen_codecTargets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES ${CANOPEN_CODEC_OD_HEADER} DESTINATION include/canopen_codec)
install(EXPORT canopen_codecTargets NAMESPACE canopen_codec:: DESTINATION share/canopen_codec/cmake)
install(FILES cmake/canopen_codecConfig.cmake DESTINATION share/canopen_codec/cmake)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  file(GLOB CANOPEN_CODEC_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  add_executable(canopen_codec_tests ${CANOPEN_CODEC_TESTS})
  robotics_apply_flags(canopen_codec_tests)
  target_link_libraries(canopen_codec_tests PRIVATE canopen_codec GTest::gtest_main)
  target_compile_definitions(canopen_codec_tests PRIVATE
    CANOPEN_CODEC_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/golden"
    CANOPEN_CODEC_EDS_PATH="${CANOPEN_CODEC_EDS}")
  include(GoogleTest)
  gtest_discover_tests(canopen_codec_tests)
endif()
