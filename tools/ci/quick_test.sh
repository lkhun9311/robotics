#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Fast inner loop: format, release build, unit tests.
set -euo pipefail
cd "$(dirname "$0")/../.."
find src -name '*.hpp' -o -name '*.cpp' | xargs --no-run-if-empty clang-format -i
colcon build --base-paths src --build-base build/release --install-base install/release
colcon test --base-paths src --build-base build/release --install-base install/release \
  --return-code-on-test-failure
colcon test-result --test-result-base build/release --verbose
