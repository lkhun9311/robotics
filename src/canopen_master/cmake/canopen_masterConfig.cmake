# SPDX-License-Identifier: Apache-2.0
# find_package(canopen_master REQUIRED) provides the imported target canopen_master::canopen_master.
include(CMakeFindDependencyMacro)
find_dependency(runtime_core)
find_dependency(canopen_codec)
include("${CMAKE_CURRENT_LIST_DIR}/canopen_masterTargets.cmake")
