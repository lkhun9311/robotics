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
# A value that cannot be read is written as "unknown" together with the reason, never dropped.
python3 - > artifacts/run-manifest.json <<'EOF'
import datetime, json, os, subprocess

manifest, errors = {}, {}
for key, cmd in {
    "git_sha": ["git", "-c", f"safe.directory={os.getcwd()}", "rev-parse", "HEAD"],
    "kernel": ["uname", "-r"],
    "gcc": ["g++", "-dumpfullversion"],
}.items():
    try:
        p = subprocess.run(cmd, capture_output=True, text=True)
        ok, reason = p.returncode == 0, (p.stderr or p.stdout).strip() or f"exit {p.returncode}"
    except OSError as e:
        ok, reason = False, str(e)
    manifest[key] = p.stdout.strip() if ok else "unknown"
    if not ok:
        errors[key] = reason
manifest["date_utc"] = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
if errors:
    manifest["errors"] = errors
print(json.dumps(manifest, indent=2))
EOF

echo "== format =="
find src -name '*.hpp' -o -name '*.cpp' | xargs --no-run-if-empty clang-format --dry-run --Werror

REL=(--build-base build/release --install-base install/release)
ASAN=(--build-base build/asan --install-base install/asan)

# Minimum discovered test counts. A run that finds fewer tests than this is red: tests went
# missing (discovery, a filter, a skipped file), and green would only mean "nothing failed".
# Raise these when adding tests.
GTEST_FLOOR=184
PYTEST_FLOOR=215
INTEGRATION_FLOOR=11
SIM_FLOOR=30

require_at_least() {
  local label=$1 found=$2 floor=$3
  if ! [[ $found =~ ^[0-9]+$ ]]; then
    echo "FAIL: $label: could not read a test count (got '$found')" >&2
    exit 1
  fi
  if [ "$found" -lt "$floor" ]; then
    echo "FAIL: $label ran $found tests, floor is $floor" >&2
    exit 1
  fi
  echo "$label: $found tests (floor $floor)"
}

# Pure C++ packages: every directory under src/ whose src/ holds .cpp files. Computed here,
# apart from the loops below, so a loop that silently skips a package is caught by its count.
mapfile -t CPP_PACKAGES < <(find src -path 'src/*/src/*' -name '*.cpp' | cut -d/ -f2 | sort -u)
if [ "${#CPP_PACKAGES[@]}" -eq 0 ]; then
  echo "FAIL: found no C++ package under src/ (expected at least runtime_core)" >&2
  exit 1
fi
echo "C++ packages: ${CPP_PACKAGES[*]}"

require_all_packages() {
  local label=$1 checked=$2
  if [ "$checked" -eq 0 ] || [ "$checked" -ne "${#CPP_PACKAGES[@]}" ]; then
    echo "FAIL: $label checked $checked packages, expected ${#CPP_PACKAGES[@]} (${CPP_PACKAGES[*]})" >&2
    exit 1
  fi
  echo "$label: $checked packages"
}

# Tests that ran and were not skipped, from a pytest JUnit XML file.
junit_count() {
  python3 -c '
import sys, xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
suites = [root] if root.tag == "testsuite" else root.iter("testsuite")
print(sum(int(s.get("tests", 0)) - int(s.get("skipped", 0)) for s in suites))' "$1"
}

# Runs the tests, prints per-test results, then fails. Printing first matters: under
# `set -e` a failing `colcon test` would end the script before the failing test is named.
run_tests() {
  local base=$1
  local rc=0 found
  colcon test --base-paths src --build-base "$base" --install-base "install/${base#build/}" \
    --return-code-on-test-failure || rc=$?
  colcon test-result --test-result-base "$base" --verbose | tee "$base/test-result.txt"
  [ "$rc" -eq 0 ] || return "$rc"
  found=$(sed -n 's/^Summary: \([0-9]*\) tests.*/\1/p' "$base/test-result.txt" | tail -1)
  require_at_least "gtest ($base)" "${found:-0}" "$GTEST_FLOOR"
}

echo "== build (release) =="
colcon build --base-paths src "${REL[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
echo "== unit tests (release) =="
run_tests build/release

echo "== simulation checks (release install, headless) =="
# The colcon setup script reads unset variables (COLCON_TRACE), so it cannot be sourced
# under `set -u` (reproduced 2026-11-14). Run the stage in a subshell with nounset off.
(
  set +u
  # shellcheck disable=SC1091
  source install/release/setup.bash
  set -u
  # -rP: a passing check's printed measurements (stamp gap, executor lag) stay in the CI log.
  python3 -m pytest -p no:cacheprovider -q -rP --junitxml=artifacts/pytest-sim.xml tools/sim_checks
)
require_at_least "simulation checks" "$(junit_count artifacts/pytest-sim.xml)" "$SIM_FLOOR"

echo "== clang-tidy (every package's src and harness; tests excluded, see plan) =="
# A package with sources but no compile database cannot be checked, which is a failure, not a skip.
tidied=0
for pkg_dir in src/*/; do
  pkg=$(basename "$pkg_dir")
  for sub in src harness; do
    [ -d "$pkg_dir$sub" ] || continue
    if [ ! -f "build/release/$pkg/compile_commands.json" ]; then
      echo "FAIL: clang-tidy: no compile_commands.json for $pkg in build/release/$pkg" >&2
      exit 1
    fi
    find "$pkg_dir$sub" -name '*.cpp' -print0 | xargs -0 --no-run-if-empty \
      clang-tidy -p "build/release/$pkg" --quiet
    if [ "$sub" = src ] && [ -n "$(find "${pkg_dir}src" -name '*.cpp' -print -quit)" ]; then
      tidied=$((tidied + 1))
    fi
  done
done
require_all_packages "clang-tidy" "$tidied"

echo "== sanitizers (ASan + UBSan, with coverage) =="
colcon build --base-paths src "${ASAN[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug "-DROBOTICS_SANITIZE=address,undefined" \
  -DROBOTICS_COVERAGE=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# The ASan build is only worth running if the flags reached the compile line of every package
# that has sources; a misspelt option would otherwise build clean, unsanitized, and pass.
sanitized=0
for pkg_dir in src/*/; do
  pkg=$(basename "$pkg_dir")
  [ -d "${pkg_dir}src" ] || continue
  if ! grep -q -e '-fsanitize=address,undefined' "build/asan/$pkg/compile_commands.json"; then
    echo "FAIL: ASan build of $pkg has no -fsanitize=address,undefined on its compile line" >&2
    exit 1
  fi
  sanitized=$((sanitized + 1))
done
require_all_packages "sanitizer flags" "$sanitized"
# Counters from an earlier run no longer match a rebuilt object (gcov "stamp mismatch"),
# and logic_sim only runs after the report, so nothing would refresh its counters.
find build/asan -name '*.gcda' -delete
run_tests build/asan

echo "== coverage report (reported, not gated) =="
# Every C++ package's own sources (src, include, harness); tests and generated headers excluded.
gcovr_args=()
for pkg in "${CPP_PACKAGES[@]}"; do
  gcovr_args+=(--filter "src/$pkg/" --exclude "src/$pkg/test/")
done
gcovr --root . "${gcovr_args[@]}" --txt artifacts/coverage.txt --xml artifacts/coverage.xml \
  "${CPP_PACKAGES[@]/#/build/asan/}"
cat artifacts/coverage.txt
for pkg in "${CPP_PACKAGES[@]}"; do
  if ! grep -q "^src/$pkg/" artifacts/coverage.txt; then
    echo "FAIL: coverage report has no file of $pkg" >&2
    exit 1
  fi
done

echo "== python tests (release harness; traces kept in artifacts/traces/release) =="
# launch_testing's pytest plugin answers pytest_ignore_collect with False ahead of pytest's own
# hook, which silently defeats --ignore (seen 2026-11-14). No test here is a launch test, and
# launch_ros (launch_testing_ros) only registers hooks that launch_testing declares, so both go.
LOGIC_SIM_BIN="$PWD/install/release/runtime_core/lib/runtime_core/logic_sim" \
ORACLE_TRACE_DIR="$PWD/artifacts/traces/release" \
  python3 -m pytest -p no:cacheprovider -p no:launch_testing -p no:launch_ros -q \
  --junitxml=artifacts/pytest.xml --ignore=tools/sim_checks tools
require_at_least "pytest (tools)" "$(junit_count artifacts/pytest.xml)" "$PYTEST_FLOOR"

echo "== oracle x harness (ASan + UBSan harness; traces in artifacts/traces/asan) =="
LOGIC_SIM_BIN="$PWD/install/asan/runtime_core/lib/runtime_core/logic_sim" \
ORACLE_TRACE_DIR="$PWD/artifacts/traces/asan" \
  python3 -m pytest -p no:cacheprovider -q --junitxml=artifacts/pytest-asan.xml \
  tools/oracle/tests/test_logic_sim_integration.py
require_at_least "oracle x harness" "$(junit_count artifacts/pytest-asan.xml)" "$INTEGRATION_FLOOR"

echo "== all checks passed =="
