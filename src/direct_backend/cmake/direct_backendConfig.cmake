# SPDX-License-Identifier: Apache-2.0
# find_package(direct_backend REQUIRED) provides the imported target direct_backend::direct_backend.
include(CMakeFindDependencyMacro)
find_dependency(runtime_core)
find_dependency(hal_contract)
find_dependency(drive_core)
include("${CMAKE_CURRENT_LIST_DIR}/direct_backendTargets.cmake")
