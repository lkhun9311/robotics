#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Usage: tools/bench/run_cyclictest.sh OUT_DIR [LOOPS]
# Baseline OS wake-up latency at 1 ms, FIFO 80, memory locked. Run alone, not next to the app.
set -euo pipefail
out_dir="$1"
loops="${2:-100000}"
mkdir -p "$out_dir"
uname -r > "$out_dir/kernel.txt"
chrt -p $$ > "$out_dir/shell-policy.txt" || true
cyclictest --mlockall --priority=80 --policy=fifo --interval=1000 \
  --loops="$loops" --histogram=2000 --histfile="$out_dir/hist.txt" \
  --quiet --json="$out_dir/cyclictest.json"
python3 "$(dirname "$0")/summarize_hist.py" "$out_dir/hist.txt" > "$out_dir/summary.json"
cat "$out_dir/summary.json"
