# SPDX-License-Identifier: Apache-2.0
# find_package(vdrive_bank REQUIRED) provides the imported target vdrive_bank::vdrive_bank.
include(CMakeFindDependencyMacro)
find_dependency(runtime_core)
find_dependency(canopen_codec)
find_dependency(drive_core)
find_dependency(socketcan_io)
find_dependency(Threads)
include("${CMAKE_CURRENT_LIST_DIR}/vdrive_bankTargets.cmake")
