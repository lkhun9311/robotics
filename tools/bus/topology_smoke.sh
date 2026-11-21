#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Topology smoke of the compose bus (B12), run on the host: the `robot` service owns the network
# namespace with vcan_host and vcan_device, and `dev` joins it.
# Proves both containers share one namespace and that a frame sent in `dev` is received in `robot`.
# Exit 0 on success.
set -euo pipefail
cd "$(dirname "$0")/../.."
compose=(docker compose)
"${compose[@]}" up -d --wait robot

robot_ns=$("${compose[@]}" exec -T robot readlink /proc/self/ns/net)
dev_ns=$("${compose[@]}" run --rm -T --no-deps dev readlink /proc/self/ns/net)
if [ "$robot_ns" != "$dev_ns" ]; then
  echo "FAIL: dev ($dev_ns) is not in robot's network namespace ($robot_ns)" >&2
  exit 1
fi

out=$(mktemp)
trap 'rm -f "$out"' EXIT
# robot waits for one frame on vcan_host; dev sends until robot has it (or 5 s pass).
"${compose[@]}" exec -T robot timeout 5 candump -L -n 1 vcan_host,5A5:7FF > "$out" &
listener=$!
"${compose[@]}" run --rm -T --no-deps dev bash -c \
  'for _ in $(seq 1 40); do cansend vcan_host 5A5#C0FFEE; sleep 0.1; done' &
sender=$!
rc=0
wait "$listener" || rc=$?
wait "$sender" || true  # The sender stops by itself after 4 s.
if [ "$rc" -ne 0 ] || ! grep -q 'vcan_host 5A5#C0FFEE' "$out"; then
  echo "FAIL: robot did not receive the frame from dev (candump exit $rc): $(cat "$out")" >&2
  exit 1
fi
echo "OK: dev and robot share $robot_ns; robot received: $(cat "$out")"
