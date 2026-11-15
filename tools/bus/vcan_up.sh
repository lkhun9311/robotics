#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Creates and raises the two virtual CAN links of the bus topology, idempotently:
#   vcan_host    the side the master (Lely peer) sits on
#   vcan_device  the side the drives sit on
# Exit 0: both links exist and are up. Exit 3 with "NOT_RUN: <reason>" on stdout when the links
# cannot be made here (no vcan module, no NET_ADMIN): a test that needs them must then fail as
# NOT_RUN, never skip. VCAN_UP_FORCE_FAIL=1 takes the NOT_RUN path on purpose.
set -uo pipefail

not_run() {
  echo "NOT_RUN: $*"
  exit 3
}

[ "${VCAN_UP_FORCE_FAIL:-0}" = 1 ] && not_run "forced by VCAN_UP_FORCE_FAIL=1"
command -v ip > /dev/null || not_run "ip (iproute2) not found"

for link in vcan_host vcan_device; do
  if ! ip link show dev "$link" > /dev/null 2>&1; then
    if ! err=$(ip link add dev "$link" type vcan 2>&1); then
      not_run "cannot create $link: $err"
    fi
  fi
  kind=$(ip -details link show dev "$link" 2>&1 | sed -n 's/^ *\(vcan\).*/\1/p' | head -1)
  [ "$kind" = vcan ] || not_run "$link exists but is not a vcan link"
  if ! err=$(ip link set dev "$link" up 2>&1); then
    not_run "cannot raise $link: $err"
  fi
done
echo "OK: vcan_host vcan_device up"
