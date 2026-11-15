# SPDX-License-Identifier: Apache-2.0
# find_package(canopen_backend REQUIRED) provides the imported target canopen_backend::canopen_backend.
include(CMakeFindDependencyMacro)
find_dependency(runtime_core)
find_dependency(canopen_codec)
find_dependency(hal_contract)
find_dependency(canopen_master)
find_dependency(socketcan_io)
find_dependency(Threads)
include("${CMAKE_CURRENT_LIST_DIR}/canopen_backendTargets.cmake")
