#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Timelines of the two delayed-RPDO2 fault rows, from their recorded observations.

Top: F1 (every RPDO2 of node 3 held 60 ms): no pair completes, the lone RPDO1 halves are dropped,
the command timeout stops the drive, and the late RPDO2s arrive after the stop has revoked the tag.
Bottom: F7 (every fourth RPDO2 held 60 ms): the late RPDO2 reaches the drive past its deadline and
is dropped not-valid while the tag is current; that seq is never actuated, the others are.

Lanes: RPDO2 as Lely sent it (vcan_host), RPDO2 and RPDO1 as the drive received them
(vcan_device), the bank's drive records for node 3, and the CiA 402 state in the statusword that
reached Lely (TPDO1 on vcan_host). Writes docs/media/e3b-fault-timeline.svg.

  python3 tools/evidence/fault_timeline.py [--out PATH]
"""
import argparse
import sys
import textwrap

import matplotlib

matplotlib.use("svg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

from common import (MEDIA, NODE, cob, decode, drive_records, frames, load_recording,  # noqa: E402
                    print_input, recording_command, rpdo_of_tag, stream_tag, wire)

INK, MUTED, GRID = "#0b0b0b", "#52514e", "#d9d8d3"
BLUE, ORANGE, AQUA, VIOLET, RED, GRAY = ("#2a78d6", "#eb6834", "#1baf7a", "#6250d6", "#e34948",
                                         "#8a8984")
T0, T1 = -25, 175  # ms around the first stream RPDO1 on vcan_device
LATE_MS = 30  # an RPDO2 later than this on vcan_device than on vcan_host was held by the rule
DROP_OTHER_TAG, DROP_NOT_VALID, DROP_MISSING_HALF = 0, 2, 3
QS_NAMES = {0: "controlword", 1: "timeout", 2: "sync loss", 3: "integrity", 4: "expiry"}
STATES = {0x27: ("Operation Enabled", AQUA), 0x07: ("Quick Stop Active", RED),
          0x23: ("Switched On", GRAY), 0x21: ("Ready to Switch On", GRAY)}
LANES = {"host2": 5, "dev2": 4, "dev1": 3, "drive": 2, "state": 1}
LANE_NAMES = {
    5: "RPDO2 sent by Lely\n(vcan_host)",
    4: "RPDO2 reaching the drive\n(vcan_device)",
    3: "RPDO1 reaching the drive\n(vcan_device)",
    2: "drive records, node 3\n(bank trace)",
    1: "CiA 402 state seen by Lely\n(TPDO1 statusword)",
}
STORY = {
    "f1": "No pair completes: the lone RPDO1s are dropped as missing halves until the command "
          "timeout stops the drive; the stop revokes the tag, so every later fragment "
          "(the late RPDO2s included) is an other-tag drop.",
    "f7": "A held RPDO2 reaches the drive past its deadline while the tag is current: dropped "
          "not valid, and its RPDO1 alone was dropped as a missing half; that seq is never "
          "actuated, the others are.",
}
ROWS = {
    "f1": "F1  test_all_late_rpdo2_times_out_to_quick_stop",
    "f7": "F7  test_late_rpdo2_is_dropped_not_valid",
}


def ms(t, t0):
    return (t - t0) * 1e3


def rule_lines(obs):
    return [ln for ln in obs["harness"]["rules"].splitlines() if ln and not ln.startswith("#")]


def panel(ax, row):
    obs = load_recording(row)
    tag = stream_tag(obs)
    r1_dev = rpdo_of_tag(obs, "device", "RPDO1", tag)
    r2_host = rpdo_of_tag(obs, "host", "RPDO2", tag)
    r2_dev = rpdo_of_tag(obs, "device", "RPDO2", tag)
    t0 = r1_dev[0][0]
    within = lambda x: T0 <= x <= T1  # noqa: E731

    sync = wire.Eds().sync_cob_id()
    for t, _, _ in frames(obs, "device", {sync}):
        if within(ms(t, t0)):
            ax.axvline(ms(t, t0), color=GRID, linewidth=0.8, zorder=0)

    host_t = {}
    for t, s, _ in r2_host:
        host_t.setdefault(s, t)
        x = ms(t, t0)
        if within(x):
            ax.plot(x, LANES["host2"], "o", color=BLUE, ms=6, zorder=3)
            ax.text(x, LANES["host2"] + 0.22, str(s), ha="center", fontsize=6.5, color=MUTED)
    first_dev = {}
    for t, s, _ in r2_dev:
        late = s in host_t and ms(t, host_t[s]) > LATE_MS and s not in first_dev
        first_dev.setdefault(s, t)
        x = ms(t, t0)
        if within(x):
            ax.plot(x, LANES["dev2"], "D" if late else "o", color=ORANGE if late else BLUE, ms=6,
                    zorder=3)
            ax.text(x, LANES["dev2"] + (-0.42 if late else 0.22), str(s), ha="center",
                    fontsize=6.5, color=MUTED)
            if late and within(ms(host_t[s], t0)):
                ax.annotate("", xy=(x, LANES["dev2"] + 0.12),
                            xytext=(ms(host_t[s], t0), LANES["host2"] - 0.12),
                            arrowprops={"arrowstyle": "->", "color": ORANGE, "lw": 1.0},
                            zorder=2)
    seen1 = set()
    for t, s, _ in r1_dev:
        x = ms(t, t0)
        if within(x) and s not in seen1:
            seen1.add(s)
            ax.plot(x, LANES["dev1"], "o", color=BLUE, ms=6, zorder=3)
            ax.text(x, LANES["dev1"] + 0.22, str(s), ha="center", fontsize=6.5, color=MUTED)

    stop_x = None
    for r in drive_records(obs):
        x = ms(r["t_ns"] / 1e9, t0)
        if not within(x):
            continue
        y = LANES["drive"]
        if r["kind"] == "actuated":
            ax.plot(x, y + 0.14, "o", color=AQUA, ms=7, zorder=3)
            ax.text(x, y + 0.34, str(r["seq"]), ha="center", fontsize=6.5, color=MUTED)
        elif r["kind"] == "dropped" and r["drop"] == DROP_NOT_VALID:
            ax.plot(x, y - 0.1, "X", color=ORANGE, ms=8, zorder=4)
        elif r["kind"] == "dropped" and r["drop"] == DROP_MISSING_HALF:
            ax.plot(x, y - 0.1, "v", color=VIOLET, ms=7, zorder=4)
        elif r["kind"] == "dropped" and r["drop"] == DROP_OTHER_TAG:
            ax.plot(x, y - 0.34, "s", color=GRAY, ms=5, zorder=3)
            ax.text(x + 1.4, y - 0.34, f"{r['count']}", ha="left", va="center", fontsize=6,
                    color=MUTED)
        elif r["kind"] == "quick_stop":
            stop_x = x
            ax.plot(x, y + 0.14, "*", color=RED, ms=13, zorder=5)
            ax.text(x + 1.5, y + 0.3, f"quick stop: {QS_NAMES.get(r['reason'], r['reason'])}",
                    fontsize=7.5, color=INK, va="bottom")

    sw = [(ms(t, t0), decode("TPDO1", d)["statusword"] & 0x6F)
          for t, _, d in frames(obs, "host", {cob("TPDO1")})]
    segments = []
    for x, state in sw:
        if segments and segments[-1][2] == state:
            segments[-1][1] = x
        else:
            segments.append([x, x, state])
    for i, (a, b, state) in enumerate(segments):
        b = segments[i + 1][0] if i + 1 < len(segments) else b
        a, b = max(a, T0), min(b, T1)
        if b <= a:
            continue
        name, color = STATES.get(state, (f"0x{state:02X}", GRAY))
        ax.barh(LANES["state"], b - a, left=a, height=0.34, color=color, alpha=0.85, zorder=2)
        if b - a > 12:
            ax.text((a + b) / 2, LANES["state"] + 0.26, name, ha="center", fontsize=7.5, color=INK)

    rules = "; ".join(rule_lines(obs))
    ax.set_title(f"{ROWS[row]}   (proxy rule: {rules})\n{textwrap.fill(STORY[row], 125)}\n"
                 f"recording: tools/interop/recorded/{row}.json.gz, node {NODE}, tag {tag}",
                 fontsize=9, color=INK, loc="left")
    ax.set_yticks(list(LANE_NAMES), [LANE_NAMES[k] for k in LANE_NAMES], fontsize=7.5)
    ax.set_ylim(0.45, 5.6)
    ax.set_xlim(T0, T1)
    ax.tick_params(colors=MUTED, labelsize=8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    return stop_x


def draw(out):
    plt.rcParams.update({"svg.hashsalt": "e3b-fault-timeline", "font.family": "DejaVu Sans"})
    fig, axes = plt.subplots(2, 1, figsize=(11, 9.2), facecolor="white", sharex=False)
    for ax, row in zip(axes, ("f1", "f7")):
        ax.set_facecolor("white")
        panel(ax, row)
    axes[1].set_xlabel("ms since the stream's first RPDO1 reached vcan_device "
                       "(CLOCK_MONOTONIC; vertical lines: SYNC on vcan_device, 10 ms apart)",
                       color=INK, fontsize=8.5)
    handles = [
        Line2D([], [], marker="o", color=BLUE, ls="", label="frame of the stream (seq above)"),
        Line2D([], [], marker="D", color=ORANGE, ls="", label="RPDO2 held 60 ms by the proxy"),
        Line2D([], [], marker="o", color=AQUA, ls="", label="actuated (seq above)"),
        Line2D([], [], marker="v", color=VIOLET, ls="", label="dropped: missing half"),
        Line2D([], [], marker="X", color=ORANGE, ls="", label="dropped: not valid"),
        Line2D([], [], marker="s", color=GRAY, ls="",
               label="dropped: other tag (count beside)"),
        Line2D([], [], marker="*", color=RED, ls="", ms=10, label="quick stop (reason)"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False, fontsize=8)
    fig.suptitle("E3b proxy fault rows on vcan: Lely master -> bus_fault_proxy -> vdrive_bank, "
                 "node 3, pdo_stream of 60 commands", fontsize=10.5, color=INK)
    fig.tight_layout(rect=(0, 0.05, 1, 0.97))
    fig.savefig(out, format="svg", facecolor="white", metadata={"Date": None})
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default=str(MEDIA / "e3b-fault-timeline.svg"))
    args = parser.parse_args(argv)
    for row in ("f1", "f7"):
        print_input(f"tools/interop/recorded/{row}.json.gz", recording_command(row))
    draw(args.out)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
