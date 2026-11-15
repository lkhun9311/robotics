#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Total travel of the last command, from its send to rest, against where its validity deadline
falls inside one control tick W, from the dump of the drive_core test
Stop.TotalTravelWithinBoundAcrossPhases (logic time, no bus). The test pins the maximum; this draws
all 20 phases and the 0.100 rad allowance. Writes docs/media/e3a-total-travel.svg.

  python3 tools/evidence/travel_phases.py [--in docs/media/e3a-total-travel.json] [--out PATH]
"""
import argparse
import json
import os
import sys

import matplotlib

matplotlib.use("svg")
import matplotlib.pyplot as plt  # noqa: E402

from common import MEDIA, ROOT, print_input, svg_metadata  # noqa: E402

DUMP = MEDIA / "e3a-total-travel.json"
SVG = MEDIA / "e3a-total-travel.svg"
TEST_BINARY = ROOT / "build" / "release" / "drive_core" / "drive_core_tests"
TEST = "Stop.TotalTravelWithinBoundAcrossPhases"
DUMP_COMMAND = ("docker compose run --rm -T dev bash -c 'DRIVE_CORE_TRAVEL_DUMP=docs/media/e3a-total-travel.json "
                f"build/release/drive_core/drive_core_tests --gtest_filter={TEST}'   "
                "# after run_checks.sh has built build/release")

INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
BAR, LIMIT = "#2a78d6", "#e34948"


def plot_data(dump):
    """Bars [(deadline phase in W in µs, travel in rad, phase index)] ordered by the deadline
    phase, the allowance and the maximum in rad, all as the dump states them."""
    bars = sorted((p["deadline_phase_in_w_ns"] / 1000, p["travel_nrad"] * 1e-9, p["phase"]) for p in dump["phases"])
    return {"bars": bars, "allowance": dump["allowance_nrad"] * 1e-9, "max": dump["max_travel_nrad"] * 1e-9}


def draw(dump, out):
    plt.rcParams.update({"svg.hashsalt": "e3a-total-travel", "font.size": 10, "font.family": "DejaVu Sans"})
    d = plot_data(dump)
    w_us = dump["w_ns"] / 1000
    width = w_us / len(d["bars"]) * 0.8
    fig, ax = plt.subplots(figsize=(8, 4.2), facecolor="white")
    ax.set_facecolor("white")
    ax.bar([x for x, _, _ in d["bars"]], [y for _, y, _ in d["bars"]], width=width, align="edge", color=BAR,
           linewidth=0)
    ax.axhline(d["allowance"], color=LIMIT, linewidth=1.5, linestyle="--")
    ax.text(w_us, d["allowance"] + 0.002, f"allowance {d['allowance']:.3f} rad (lease 48 ms at v_max, "
            "plus the brake at a_stop)", color=LIMIT, ha="right", va="bottom", fontsize=8.5)
    for x, y, _ in d["bars"]:
        if y == d["max"] or y == min(v for _, v, _ in d["bars"]):
            ax.text(x + width / 2, y / 2, f"{y:.3f}", color="white", ha="center", va="center", rotation=90,
                    fontsize=8)
    ax.set_xlim(0, w_us)
    ax.set_ylim(0, 0.115)
    ax.set_xlabel(f"validity deadline phase inside one control tick W = {w_us / 1000:.0f} ms (µs)", color=INK)
    ax.set_ylabel("total travel, send to rest (rad)", color=INK)
    ax.set_title(f"drive_core, logic time: last command delivered 20 ms late, stopped by the expiry guard\n"
                 f"{len(d['bars'])} phases, maximum {d['max']:.3f} rad against the {d['allowance']:.3f} rad "
                 f"allowance ({TEST})", color=INK, fontsize=10)
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
    parser.add_argument("--in", dest="src", default=str(DUMP))
    parser.add_argument("--out", default=str(SVG))
    args = parser.parse_args(argv)
    print_input(os.path.relpath(args.src, ROOT), DUMP_COMMAND)
    with open(args.src, encoding="utf-8") as f:
        dump = json.load(f)
    travel = [p["travel_nrad"] for p in dump["phases"]]
    if len(travel) != 20 or max(travel) != dump["max_travel_nrad"]:
        raise SystemExit(f"{args.src}: expected 20 phases whose maximum is max_travel_nrad")
    draw(dump, args.out)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
