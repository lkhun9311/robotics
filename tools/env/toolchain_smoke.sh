#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Fails on the first missing tool or capability. EXTRA_TOOL lets a test demand a tool that
# does not exist, to prove the failure is visible.
set -euo pipefail
for tool in g++ cmake colcon python3 cyclictest clang-format clang-tidy gdb valgrind gcovr ip chrt ${EXTRA_TOOL:-}; do
  command -v "$tool" >/dev/null || { echo "MISSING $tool" >&2; exit 1; }
done
cyclictest --help 2>&1 | grep -q -- "--json" || { echo "cyclictest lacks --json" >&2; exit 1; }
test -f /usr/lib/x86_64-linux-gnu/libgtest.a || { echo "libgtest.a missing" >&2; exit 1; }
g++ --version | head -1
cmake --version | head -1
python3 --version
echo "toolchain ok"
