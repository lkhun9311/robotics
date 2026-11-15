# SPDX-License-Identifier: Apache-2.0
# find_package(socketcan_io REQUIRED) provides the imported target socketcan_io::socketcan_io.
include(CMakeFindDependencyMacro)
find_dependency(runtime_core)
find_dependency(canopen_codec)
include("${CMAKE_CURRENT_LIST_DIR}/socketcan_ioTargets.cmake")
