#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Host step of the CI vcan gate (B13), run on the runner before the checks container starts:
# loads the vcan module and proves a vcan link can be created here.
# Always writes the status file (default artifacts/preflight/vcan-status.txt): "OK", or "NOT_RUN:
# <reason>" and exit 3.
# The status is uploaded by the workflow whether or not this step fails.
# VCAN_PREFLIGHT_FORCE_FAIL=1 takes the NOT_RUN path on purpose.
set -uo pipefail
out=${1:-artifacts/preflight/vcan-status.txt}
mkdir -p "$(dirname "$out")"
status="NOT_RUN: the preflight ended before a verdict"
trap 'echo "$status" > "$out"; echo "vcan preflight: $status"' EXIT

not_run() {
  status="NOT_RUN: $*"
  exit 3
}

[ "${VCAN_PREFLIGHT_FORCE_FAIL:-0}" = 1 ] && not_run "forced by VCAN_PREFLIGHT_FORCE_FAIL=1"
as_root=()
[ "$(id -u)" = 0 ] || as_root=(sudo -n)
if [ ! -d /sys/module/vcan ]; then
  err=$("${as_root[@]}" modprobe vcan 2>&1) || not_run "modprobe vcan: $err"
fi
probe=vcan_preflight
err=$("${as_root[@]}" ip link add dev "$probe" type vcan 2>&1) || not_run "cannot create a vcan link: $err"
"${as_root[@]}" ip link del dev "$probe" || not_run "cannot delete the probe link $probe"
status="OK"
