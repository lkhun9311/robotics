# SPDX-License-Identifier: Apache-2.0
# find_package(hal_contract REQUIRED) provides the imported target hal_contract::hal_contract.
include(CMakeFindDependencyMacro)
find_dependency(runtime_core)
include("${CMAKE_CURRENT_LIST_DIR}/hal_contractTargets.cmake")
