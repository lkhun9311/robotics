# SPDX-License-Identifier: Apache-2.0
# find_package(drive_core REQUIRED) provides the imported target drive_core::drive_core.
include(CMakeFindDependencyMacro)
find_dependency(runtime_core)
find_dependency(canopen_codec)
include("${CMAKE_CURRENT_LIST_DIR}/drive_coreTargets.cmake")
