#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Histogram of the bus_fault_proxy normal-path cost from the proxy cost JSON.

The JSON (tools/bench/proxy_cost.py) holds the exact distribution (`dist`: [value ns, count]);
this bins it, marks p50, p99 and max as the JSON states them, and writes an SVG. A measurement on
the named host, not a bound.

  python3 tools/evidence/proxy_cost_histogram.py [--in docs/media/e3b-proxy-cost.json]
      [--out docs/media/e3b-proxy-cost.svg] [--bin-ns 200]
"""
import argparse
import json
import os
import sys

import matplotlib

matplotlib.use("svg")
import matplotlib.pyplot as plt  # noqa: E402

from common import MEDIA, PROXY_COST_COMMAND, ROOT, print_input, svg_metadata  # noqa: E402

INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
BAR, MARK = "#2a78d6", "#eb6834"


def plot_data(cost, bin_ns):
    """What the histogram plots: bars [(bin start in µs, frames)] ascending and the marker
    positions {p50, p99, max} in µs, as the JSON states them."""
    counts = {}
    for value, n in cost["dist"]:
        counts[value // bin_ns] = counts.get(value // bin_ns, 0) + n
    bars = [(b * bin_ns / 1000, counts[b]) for b in sorted(counts)]
    return {"bars": bars, "markers": {name: cost[name] / 1000 for name in ("p50", "p99", "max")}}


def draw(cost, out, bin_ns):
    plt.rcParams.update({"svg.hashsalt": "e3b-proxy-cost", "font.size": 10,
                         "font.family": "DejaVu Sans"})
    data = plot_data(cost, bin_ns)
    fig, ax = plt.subplots(figsize=(8, 4.2), facecolor="white")
    ax.set_facecolor("white")
    ax.bar([x for x, _ in data["bars"]], [n for _, n in data["bars"]], width=bin_ns / 1000 * 0.9, align="edge", color=BAR,
           linewidth=0)
    ax.set_yscale("log")
    top = max(n for _, n in data["bars"])
    for name, x in data["markers"].items():
        ax.axvline(x, color=MARK, linewidth=1.5, linestyle="--")
        ax.text(x, top * 1.6, f" {name} {x:.2f} µs", color=INK, va="bottom", ha="left",
                fontsize=9)
    ax.set_ylim(0.7, top * 6)
    ax.set_xlim(0, cost["max"] / 1000 * 1.08)
    ax.set_xlabel("cost per forwarded frame: recvmsg return to write return (µs)", color=INK)
    ax.set_ylabel(f"frames per {bin_ns} ns bin (log scale)", color=INK)
    ax.set_title(f"bus_fault_proxy normal-path cost on host {cost['host']}: "
                 f"{cost['count']} frames over vcan\n(a measurement on one host, not a bound; "
                 f"commit {cost['commit'][:7]})", color=INK, fontsize=10.5)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED)
    fig.tight_layout()
    fig.savefig(out, format="svg", facecolor="white", metadata=svg_metadata())
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--in", dest="src", default=str(MEDIA / "e3b-proxy-cost.json"))
    parser.add_argument("--out", default=str(MEDIA / "e3b-proxy-cost.svg"))
    parser.add_argument("--bin-ns", type=int, default=200)
    args = parser.parse_args(argv)
    print_input(os.path.relpath(args.src, ROOT), PROXY_COST_COMMAND)
    with open(args.src, encoding="utf-8") as f:
        cost = json.load(f)
    if sum(n for _, n in cost["dist"]) != cost["samples"]:
        raise SystemExit(f"{args.src}: dist does not sum to the {cost['samples']} samples")
    draw(cost, args.out, args.bin_ns)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
