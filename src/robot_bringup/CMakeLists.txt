# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(robot_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
