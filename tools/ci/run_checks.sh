#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Every repository check. CI and local runs use this same script inside the dev image.
set -eo pipefail
cd "$(dirname "$0")/../.."
# ROS setup scripts read unset variables, so `set -u` comes only after sourcing.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
mkdir -p artifacts
{
  echo "{"
  echo "  \"git_sha\": \"$(git -c safe.directory="$PWD" rev-parse HEAD 2>/dev/null || echo unknown)\","
  echo "  \"kernel\": \"$(uname -r)\","
  echo "  \"gcc\": \"$(g++ -dumpfullversion)\","
  echo "  \"date_utc\": \"$(date -u +%Y-%m-%dT%H:%M:%SZ)\""
  echo "}"
} > artifacts/run-manifest.json

echo "== format =="
find src -name '*.hpp' -o -name '*.cpp' | xargs --no-run-if-empty clang-format --dry-run --Werror

REL=(--build-base build/release --install-base install/release)
ASAN=(--build-base build/asan --install-base install/asan)

# Runs the tests, prints per-test results, then fails. Printing first matters: under
# `set -e` a failing `colcon test` would end the script before the failing test is named.
run_tests() {
  local base=$1
  local rc=0
  colcon test --base-paths src --build-base "$base" --install-base "install/${base#build/}" \
    --return-code-on-test-failure || rc=$?
  colcon test-result --test-result-base "$base" --verbose
  return "$rc"
}

echo "== build (release) =="
colcon build --base-paths src "${REL[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
echo "== unit tests (release) =="
run_tests build/release

echo "== clang-tidy (src and harness; tests excluded, see plan) =="
find src/runtime_core/src -name '*.cpp' -print0 | xargs -0 --no-run-if-empty \
  clang-tidy -p build/release/runtime_core --quiet
if [ -d src/runtime_core/harness ]; then
  find src/runtime_core/harness -name '*.cpp' -print0 | xargs -0 --no-run-if-empty \
    clang-tidy -p build/release/runtime_core --quiet
fi

echo "== sanitizers (ASan + UBSan, with coverage) =="
colcon build --base-paths src "${ASAN[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug "-DRUNTIME_CORE_SANITIZE=address,undefined" \
  -DRUNTIME_CORE_COVERAGE=ON
# Counters from an earlier run no longer match a rebuilt object (gcov "stamp mismatch"),
# and logic_sim only runs after the report, so nothing would refresh its counters.
find build/asan -name '*.gcda' -delete
run_tests build/asan

echo "== coverage report (reported, not gated) =="
gcovr --root src/runtime_core --exclude 'src/runtime_core/test/.*' \
  --txt artifacts/coverage.txt --xml artifacts/coverage.xml build/asan/runtime_core
cat artifacts/coverage.txt

echo "== python tests (release harness; traces kept in artifacts/traces/release) =="
LOGIC_SIM_BIN="$PWD/install/release/runtime_core/lib/runtime_core/logic_sim" \
ORACLE_TRACE_DIR="$PWD/artifacts/traces/release" \
  python3 -m pytest -p no:cacheprovider -q --junitxml=artifacts/pytest.xml tools

echo "== oracle x harness (ASan + UBSan harness; traces in artifacts/traces/asan) =="
LOGIC_SIM_BIN="$PWD/install/asan/runtime_core/lib/runtime_core/logic_sim" \
ORACLE_TRACE_DIR="$PWD/artifacts/traces/asan" \
  python3 -m pytest -p no:cacheprovider -q --junitxml=artifacts/pytest-asan.xml \
  tools/oracle/tests/test_logic_sim_integration.py

echo "== all checks passed =="
