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
GTEST_FLOOR=346
PYTEST_FLOOR=245
INTEGRATION_FLOOR=11
SIM_FLOOR=32
VCAN_FLOOR=152
TSAN_FLOOR=33

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

# Every production source of a package (its src/ directory) must be compiled with the sanitizer
# flags: one sanitized test file must not stand in for an unsanitized library. Fails naming each
# file without them, and fails when the database has no entry under the directory at all.
# Third argument: the flag (default the ASan + UBSan one).
require_sanitized() {
  python3 - "$1" "$2" "${3:--fsanitize=address,undefined}" <<'EOF'
import json, os, shlex, sys

src_dir, db = os.path.realpath(sys.argv[1]), sys.argv[2]
flag = sys.argv[3]
try:
    with open(db) as f:
        entries = json.load(f)
except (OSError, ValueError) as e:
    sys.exit(f"FAIL: sanitizer flags: cannot read {db}: {e}")
checked, missing = 0, []
for entry in entries:
    path = os.path.realpath(os.path.join(entry.get("directory", ""), entry["file"]))
    if os.path.commonpath([path, src_dir]) != src_dir:
        continue
    checked += 1
    args = entry["arguments"] if "arguments" in entry else shlex.split(entry["command"])
    if flag not in args:
        missing.append(path)
for path in missing:
    print(f"FAIL: ASan build compiles {path} without {flag}", file=sys.stderr)
if checked == 0:
    sys.exit(f"FAIL: sanitizer flags: {db} has no entry under {src_dir}")
if missing:
    sys.exit(1)
print(f"sanitizer flags: {checked} sources under {sys.argv[1]}")
EOF
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
  # "Summary: N tests, E errors, F failures, S skipped". A skipped test did not run: any skip is
  # a failure, and the floor counts only the tests that ran.
  local summary total skipped
  summary=$(grep '^Summary: ' "$base/test-result.txt" | tail -1)
  total=$(sed -n 's/^Summary: \([0-9]*\) tests.*/\1/p' <<<"$summary")
  skipped=$(sed -n 's/.* \([0-9]*\) skipped.*/\1/p' <<<"$summary")
  if ! [[ $total =~ ^[0-9]+$ && $skipped =~ ^[0-9]+$ ]]; then
    echo "FAIL: gtest ($base): could not read the summary (got '$summary')" >&2
    exit 1
  fi
  if [ "$skipped" -ne 0 ]; then
    echo "FAIL: gtest ($base): $skipped tests skipped; a skipped test did not run" >&2
    exit 1
  fi
  found=$((total - skipped))
  require_at_least "gtest ($base)" "$found" "$GTEST_FLOOR"
}

echo "== build (release) =="
colcon build --base-paths src "${REL[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
echo "== exported targets (an outside consumer at the compiler's default C++ standard) =="
# The consumer sets no standard of its own: whatever its headers need must come from the
# imported targets (INTERFACE_COMPILE_FEATURES). It includes every installed public header.
consumer=build/release/export_consumer
rm -rf "$consumer"
mkdir -p "$consumer"
cat > "$consumer/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.22)
project(export_consumer LANGUAGES CXX)
find_package(drive_core REQUIRED)
find_package(socketcan_io REQUIRED)
find_package(vdrive_bank REQUIRED)
find_package(bus_fault_proxy REQUIRED)
find_package(hal_contract REQUIRED)
find_package(direct_backend REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE drive_core::drive_core socketcan_io::socketcan_io
  vdrive_bank::vdrive_bank bus_fault_proxy::bus_fault_proxy hal_contract::hal_contract
  direct_backend::direct_backend)
EOF
{
  # hal_contract/testing/ holds the gtest contract suite, which every backend's test binary
  # compiles and links against gtest; it is not a library header.
  for pkg in runtime_core canopen_codec drive_core socketcan_io vdrive_bank bus_fault_proxy hal_contract direct_backend; do
    (cd "install/release/$pkg/include" && find . -name '*.hpp' -not -path './hal_contract/testing/*' | sort |
      sed 's|^\./\(.*\)|#include "\1"|')
  done
  echo 'int main() { return 0; }'
} > "$consumer/main.cpp"
(
  set +u
  # shellcheck disable=SC1091
  source install/release/setup.bash
  set -u
  cmake -S "$consumer" -B "$consumer/build" -DCMAKE_BUILD_TYPE=Release > "$consumer/configure.log"
  cmake --build "$consumer/build"
)

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

echo "== clang-tidy (every package's src and harness; tests are not linted) =="
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
  require_sanitized "${pkg_dir}src" "build/asan/$pkg/compile_commands.json"
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
# tools/interop needs vcan and NET_ADMIN; it is not collected here (B13).
# launch_testing's pytest plugin answers pytest_ignore_collect with False ahead of pytest's own
# hook, which silently defeats --ignore (seen 2026-11-14). No test here is a launch test, and
# launch_ros (launch_testing_ros) only registers hooks that launch_testing declares, so both go.
LOGIC_SIM_BIN="$PWD/install/release/runtime_core/lib/runtime_core/logic_sim" \
ORACLE_TRACE_DIR="$PWD/artifacts/traces/release" \
  python3 -m pytest -p no:cacheprovider -p no:launch_testing -p no:launch_ros -q \
  --junitxml=artifacts/pytest.xml --ignore=tools/sim_checks --ignore=tools/interop tools
require_at_least "pytest (tools)" "$(junit_count artifacts/pytest.xml)" "$PYTEST_FLOOR"

echo "== oracle x harness (ASan + UBSan harness; traces in artifacts/traces/asan) =="
LOGIC_SIM_BIN="$PWD/install/asan/runtime_core/lib/runtime_core/logic_sim" \
ORACLE_TRACE_DIR="$PWD/artifacts/traces/asan" \
  python3 -m pytest -p no:cacheprovider -q --junitxml=artifacts/pytest-asan.xml \
  tools/oracle/tests/test_logic_sim_integration.py
require_at_least "oracle x harness" "$(junit_count artifacts/pytest-asan.xml)" "$INTEGRATION_FLOOR"

echo "== vcan integration (tools/interop: processes on vcan_host/vcan_device) =="
# Needs NET_ADMIN (and the vcan module on the host). The status file is written by a trap that
# covers the whole stage: OK, NOT_RUN: <reason> (the stage could not run, which fails the check;
# never a skip), or FAIL: <what>. The subshell keeps the trap to this stage.
(
  verdict="FAIL: the vcan stage ended before a verdict"
  trap 'echo "$verdict" > artifacts/vcan-status.txt; echo "vcan status: $verdict"' EXIT
  rc=0
  out=$(tools/bus/vcan_up.sh) || rc=$?
  if [ "$rc" -eq 3 ]; then
    verdict=$out
    exit 1
  elif [ "$rc" -ne 0 ]; then
    verdict="FAIL: vcan_up.sh exited $rc: $out"
    exit 1
  fi
  verdict="FAIL: interop tests"
  set +u
  # shellcheck disable=SC1091
  source install/release/setup.bash
  set -u
  # Every judged observation (passed or failed) is kept under artifacts/interop, one gzipped
  # JSON per row (about 30 kB each), so a CI failure can be judged again from its artifact.
  rm -rf artifacts/interop
  INTEROP_RECORD_DIR=artifacts/interop \
    python3 -m pytest -p no:cacheprovider -p no:launch_testing -p no:launch_ros -q -rP \
    --junitxml=artifacts/pytest-vcan.xml tools/interop
  verdict="FAIL: interop test count"
  require_at_least "vcan integration" "$(junit_count artifacts/pytest-vcan.xml)" "$VCAN_FLOOR"
  verdict="OK"
)

echo "== thread sanitizer (vdrive_bank: control thread, trace ring and writer thread) =="
TSAN=(--build-base build/tsan --install-base install/tsan)
colcon build --base-paths src "${TSAN[@]}" --packages-up-to vdrive_bank \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DROBOTICS_SANITIZE=thread -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
require_sanitized src/vdrive_bank/src build/tsan/vdrive_bank/compile_commands.json -fsanitize=thread
(
  export TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1
  rc=0
  colcon test --base-paths src "${TSAN[@]}" --packages-select vdrive_bank \
    --return-code-on-test-failure || rc=$?
  colcon test-result --test-result-base build/tsan/vdrive_bank --verbose | tee build/tsan/test-result.txt
  [ "$rc" -eq 0 ] || exit "$rc"
  summary=$(grep '^Summary: ' build/tsan/test-result.txt | tail -1)
  total=$(sed -n 's/^Summary: \([0-9]*\) tests.*/\1/p' <<<"$summary")
  skipped=$(sed -n 's/.* \([0-9]*\) skipped.*/\1/p' <<<"$summary")
  if [ "${skipped:-x}" != 0 ]; then
    echo "FAIL: tsan: skipped tests or unreadable summary ('$summary')" >&2
    exit 1
  fi
  require_at_least "tsan (vdrive_bank)" "$total" "$TSAN_FLOOR"
)

echo "== all checks passed =="
