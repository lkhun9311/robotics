#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Fast inner loop: format, release build, unit tests.
set -euo pipefail
cd "$(dirname "$0")/../.."
find src -name '*.hpp' -o -name '*.cpp' | xargs --no-run-if-empty clang-format -i
colcon build --base-paths src --build-base build/release --install-base install/release
# Print per-test results before failing, so the failing test is named (see run_checks.sh).
rc=0
colcon test --base-paths src --build-base build/release --install-base install/release \
  --return-code-on-test-failure || rc=$?
colcon test-result --test-result-base build/release --verbose
exit "$rc"
