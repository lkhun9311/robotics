#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Summarise a cyclictest --histfile into percentiles. Overflowed samples make percentiles lower bounds."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def parse_histogram(text: str) -> tuple[list[tuple[int, int]], dict[str, list[int]]]:
    buckets: list[tuple[int, int]] = []
    meta: dict[str, list[int]] = {}
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("#"):
            if ":" in line:
                key, _, rest = line[1:].partition(":")
                numbers = [int(tok) for tok in rest.split() if tok.isdigit()]
                meta[key.strip()] = numbers
            continue
        columns = [int(tok) for tok in line.split()]
        buckets.append((columns[0], sum(columns[1:])))
    return buckets, meta


def percentile(buckets: list[tuple[int, int]], q: float) -> int | None:
    total = sum(count for _, count in buckets)
    if total == 0:
        return None
    needed = q * total
    running = 0
    for latency_us, count in buckets:
        running += count
        if running >= needed:
            return latency_us
    return buckets[-1][0]


def summarize(text: str) -> dict:
    buckets, meta = parse_histogram(text)
    in_histogram = sum(count for _, count in buckets)
    if in_histogram == 0:
        raise ValueError("empty histogram: no samples were recorded")
    overflows = sum(meta.get("Histogram Overflows", []))
    max_values = meta.get("Max Latencies", [])
    return {
        "samples_in_histogram": in_histogram,
        "overflows": overflows,
        "samples_total": in_histogram + overflows,
        "p50_us": percentile(buckets, 0.50),
        "p99_us": percentile(buckets, 0.99),
        "p999_us": percentile(buckets, 0.999),
        "max_us": max(max_values) if max_values else None,
        "percentiles_are_lower_bounds": overflows > 0,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("histfile")
    args = parser.parse_args(argv)
    try:
        summary = summarize(Path(args.histfile).read_text(encoding="utf-8"))
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
