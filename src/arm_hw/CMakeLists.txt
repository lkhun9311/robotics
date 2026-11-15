# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(arm_hw LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/robotics_flags.cmake)
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(runtime_core REQUIRED)
find_package(hal_contract REQUIRED)
find_package(direct_backend REQUIRED)
find_package(canopen_codec REQUIRED)
find_package(canopen_master REQUIRED)
find_package(canopen_backend REQUIRED)
find_package(socketcan_io REQUIRED)

# The plugin library: both SystemInterface plugins. The pure libraries are linked in statically and
# their symbols kept local (--exclude-libs), so nothing in a host process binds to them.
file(GLOB ARM_HW_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(arm_hw SHARED ${ARM_HW_SOURCES})
robotics_apply_flags(arm_hw)
target_compile_features(arm_hw PUBLIC cxx_std_20)
target_include_directories(arm_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(arm_hw PUBLIC
  hardware_interface::hardware_interface pluginlib::pluginlib rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  runtime_core::runtime_core hal_contract::hal_contract direct_backend::direct_backend
  canopen_codec::canopen_codec canopen_master::canopen_master canopen_backend::canopen_backend
  socketcan_io::socketcan_io ${CMAKE_DL_LIBS})
target_link_options(arm_hw PRIVATE -Wl,--exclude-libs,ALL -Wl,--no-undefined)

pluginlib_export_plugin_description_file(hardware_interface arm_hw_plugins.xml)

install(TARGETS arm_hw LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  find_package(drive_core REQUIRED)
  find_package(class_loader REQUIRED)
  include(GoogleTest)
  # The component tests link the plugin library and drive it directly or through a
  # ResourceManager, over an in-memory drive bus (test/drive_bus_transport.hpp).
  add_executable(arm_hw_tests test/test_arm_system.cpp test/main.cpp)
  robotics_apply_flags(arm_hw_tests)
  target_link_libraries(arm_hw_tests PRIVATE arm_hw drive_core::drive_core GTest::gtest)
  gtest_discover_tests(arm_hw_tests DISCOVERY_TIMEOUT 30)
  # The unload test does not link the plugin library: it loads it through class_loader, as
  # pluginlib does.
  add_executable(arm_hw_unload_tests test/test_unload.cpp test/main.cpp)
  robotics_apply_flags(arm_hw_unload_tests)
  target_link_libraries(arm_hw_unload_tests PRIVATE
    hardware_interface::hardware_interface class_loader::class_loader rclcpp::rclcpp
    hal_contract::hal_contract canopen_backend::canopen_backend canopen_master::canopen_master
    canopen_codec::canopen_codec drive_core::drive_core GTest::gtest ${CMAKE_DL_LIBS})
  target_compile_definitions(arm_hw_unload_tests PRIVATE
    ARM_HW_LIBRARY="$<TARGET_FILE:arm_hw>")
  add_dependencies(arm_hw_unload_tests arm_hw)
  gtest_discover_tests(arm_hw_unload_tests DISCOVERY_TIMEOUT 30)
endif()

ament_package()
