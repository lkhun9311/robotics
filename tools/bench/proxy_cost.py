#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The bus_fault_proxy normal-path cost (B10) from its trace, as JSON.

The proxy measures each frame it forwards on the normal path (nothing held, no rule acting):
CLOCK_MONOTONIC right after recvmsg to right after write returns, in ns, and writes the
summary (count, p50, p99, max; nearest rank) into its exit record. This reads that record. It
is a published measurement, not a threshold.
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
from pathlib import Path

DEFINITION = ("bus_fault_proxy normal path, per forwarded frame: CLOCK_MONOTONIC after recvmsg "
              "returns to after write returns")


def summarize(trace: Path) -> dict:
    records = [json.loads(line) for line in Path(trace).read_text().splitlines() if line]
    if not records or records[-1].get("event") != "exit":
        raise ValueError(f"{trace}: the trace does not end with the proxy's exit record")
    last = records[-1]
    if last.get("trace_overflow") != 0:
        raise ValueError(f"{trace}: trace_overflow = {last.get('trace_overflow')}")
    cost = last["cost_ns"]
    return {"definition": DEFINITION, "unit": "ns", "count": cost["count"],
            "samples": cost["samples"], "p50": cost["p50"], "p99": cost["p99"],
            "max": cost["max"]}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("trace", type=Path, help="the proxy's --trace file")
    parser.add_argument("--out", type=Path, default=Path("artifacts/proxy-cost.json"))
    parser.add_argument("--commit", default=None, help="the measured commit (SHA)")
    parser.add_argument("--host", default=None, help="the machine (default: this host name)")
    args = parser.parse_args(argv)
    out = summarize(args.trace)
    out["commit"] = args.commit
    out["host"] = args.host or socket.gethostname()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
