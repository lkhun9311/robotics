# SPDX-License-Identifier: Apache-2.0
# Build flags shared by every pure C++ package (runtime_core, canopen_codec, drive_core).
# Include it, then call robotics_apply_flags(<target>) for each library or executable.
include_guard(GLOBAL)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(ROBOTICS_SANITIZE "" CACHE STRING "Comma-separated -fsanitize list, empty to disable")
option(ROBOTICS_COVERAGE "Instrument for gcov coverage" OFF)

# Warnings and compile-time instrumentation are private to the target; the link options are
# PUBLIC, so whoever links the target (an exported one included) links the sanitizer runtime
# and the gcov library too.
function(robotics_apply_flags target)
  target_compile_options(${target} PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
  if(ROBOTICS_SANITIZE)
    target_compile_options(${target} PRIVATE
      -fsanitize=${ROBOTICS_SANITIZE} -fno-omit-frame-pointer)
    target_link_options(${target} PUBLIC -fsanitize=${ROBOTICS_SANITIZE})
  endif()
  if(ROBOTICS_COVERAGE)
    target_compile_options(${target} PRIVATE --coverage)
    target_link_options(${target} PUBLIC --coverage)
  endif()
endfunction()
