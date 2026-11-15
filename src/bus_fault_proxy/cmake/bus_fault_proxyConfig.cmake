# SPDX-License-Identifier: Apache-2.0
# find_package(bus_fault_proxy REQUIRED) provides the imported target bus_fault_proxy::bus_fault_proxy.
include(CMakeFindDependencyMacro)
find_dependency(runtime_core)
find_dependency(canopen_codec)
include("${CMAKE_CURRENT_LIST_DIR}/bus_fault_proxyTargets.cmake")
