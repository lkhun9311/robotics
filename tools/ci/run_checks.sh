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

echo "== build (release) =="
colcon build --base-paths src "${REL[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
echo "== unit tests (release) =="
colcon test --base-paths src "${REL[@]}" --return-code-on-test-failure
colcon test-result --test-result-base build/release --verbose

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
colcon test --base-paths src "${ASAN[@]}" --return-code-on-test-failure
colcon test-result --test-result-base build/asan --verbose

echo "== coverage report (reported, not gated) =="
gcovr --root src/runtime_core --exclude 'src/runtime_core/test/.*' \
  --txt artifacts/coverage.txt --xml artifacts/coverage.xml build/asan/runtime_core
cat artifacts/coverage.txt

echo "== python tests =="
python3 -m pytest -p no:cacheprovider -q --junitxml=artifacts/pytest.xml tools

echo "== all checks passed =="
