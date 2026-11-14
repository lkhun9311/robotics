# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(ur5e_description NONE)

find_package(ament_cmake REQUIRED)

# The Menagerie model is not vendored: the dev image installs the pinned, verified copy and
# points MENAGERIE_UR5E_DIR at it. Without it the package would install a scene that cannot load.
set(MENAGERIE_UR5E_DIR "$ENV{MENAGERIE_UR5E_DIR}")
if(NOT MENAGERIE_UR5E_DIR)
  message(FATAL_ERROR "MENAGERIE_UR5E_DIR is not set; build inside the dev image")
endif()
if(NOT EXISTS "${MENAGERIE_UR5E_DIR}/ur5e.xml")
  message(FATAL_ERROR "MENAGERIE_UR5E_DIR='${MENAGERIE_UR5E_DIR}' has no ur5e.xml")
endif()

install(FILES "${MENAGERIE_UR5E_DIR}/ur5e.xml" "${MENAGERIE_UR5E_DIR}/LICENSE"
  mjcf/scene_baseline.xml
  DESTINATION share/${PROJECT_NAME}/mjcf)
install(DIRECTORY "${MENAGERIE_UR5E_DIR}/assets" DESTINATION share/${PROJECT_NAME}/mjcf)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_package()
