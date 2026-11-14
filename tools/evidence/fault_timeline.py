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
                    print_input, recording_command, rpdo_of_tag, stream_tag, svg_metadata,
                    wire)

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
          "timeout stops the drive; the stop revokes the tag, so a later fragment (a late RPDO2 "
          "included) that a SYNC closes on is an other-tag drop; frames after the bus's last "
          "SYNC are never closed and are named by the judge as trailing.",
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


def panel_data(row):
    """Everything a panel plots, as data: {tag, rules, syncs: [x], marks: [{lane, x, kind,
    label}], states: [(from x, to x, CiA 402 state)], arrows: [(host x, device x, seq)]}, x in
    ms since the stream's first RPDO1 on vcan_device, only what lies in [T0, T1]."""
    obs = load_recording(row)
    tag = stream_tag(obs)
    r1_dev = rpdo_of_tag(obs, "device", "RPDO1", tag)
    r2_host = rpdo_of_tag(obs, "host", "RPDO2", tag)
    r2_dev = rpdo_of_tag(obs, "device", "RPDO2", tag)
    t0 = r1_dev[0][0]
    within = lambda x: T0 <= x <= T1  # noqa: E731
    out = {"tag": tag, "rules": rule_lines(obs), "syncs": [], "marks": [], "states": [],
           "arrows": []}
    sync = wire.Eds().sync_cob_id()
    out["syncs"] = [ms(t, t0) for t, _, _ in frames(obs, "device", {sync}) if within(ms(t, t0))]

    def mark(lane, x, kind, label=""):
        out["marks"].append({"lane": lane, "x": x, "kind": kind, "label": label})

    host_t = {}
    for t, s, _ in r2_host:
        host_t.setdefault(s, t)
        if within(ms(t, t0)):
            mark("host2", ms(t, t0), "frame", s)
    first_dev = {}
    for t, s, _ in r2_dev:
        late = s in host_t and ms(t, host_t[s]) > LATE_MS and s not in first_dev
        first_dev.setdefault(s, t)
        x = ms(t, t0)
        if within(x):
            mark("dev2", x, "late" if late else "frame", s)
            if late and within(ms(host_t[s], t0)):
                out["arrows"].append((ms(host_t[s], t0), x, s))
    seen1 = set()
    for t, s, _ in r1_dev:
        if within(ms(t, t0)) and s not in seen1:
            seen1.add(s)
            mark("dev1", ms(t, t0), "frame", s)
    drops = {DROP_NOT_VALID: "not_valid", DROP_MISSING_HALF: "missing_half",
             DROP_OTHER_TAG: "other_tag"}
    for r in drive_records(obs):
        x = ms(r["t_ns"] / 1e9, t0)
        if not within(x):
            continue
        if r["kind"] == "actuated":
            mark("drive", x, "actuated", r["seq"])
        elif r["kind"] == "dropped" and r["drop"] in drops:
            mark("drive", x, drops[r["drop"]], r["count"])
        elif r["kind"] == "quick_stop":
            mark("drive", x, "quick_stop", QS_NAMES.get(r["reason"], r["reason"]))

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
        if b > a:
            out["states"].append((a, b, state))
    return out


STYLE = {  # (marker, color, size, y offset, z) per mark kind and lane
    "frame": ("o", BLUE, 6, 0.0, 3), "late": ("D", ORANGE, 6, 0.0, 3),
    "actuated": ("o", AQUA, 7, 0.14, 3), "not_valid": ("X", ORANGE, 8, -0.1, 4),
    "missing_half": ("v", VIOLET, 7, -0.1, 4), "other_tag": ("s", GRAY, 5, -0.34, 3),
    "quick_stop": ("*", RED, 13, 0.14, 5),
}


def panel(ax, row):
    data = panel_data(row)
    for x in data["syncs"]:
        ax.axvline(x, color=GRID, linewidth=0.8, zorder=0)
    for xh, xd, _ in data["arrows"]:
        ax.annotate("", xy=(xd, LANES["dev2"] + 0.12), xytext=(xh, LANES["host2"] - 0.12),
                    arrowprops={"arrowstyle": "->", "color": ORANGE, "lw": 1.0}, zorder=2)
    for m in data["marks"]:
        marker, color, size, dy, z = STYLE[m["kind"]]
        x, y = m["x"], LANES[m["lane"]] + dy
        ax.plot(x, y, marker, color=color, ms=size, zorder=z)
        if m["kind"] == "other_tag":
            ax.text(x + 1.4, y, str(m["label"]), ha="left", va="center", fontsize=6, color=MUTED)
        elif m["kind"] == "quick_stop":
            ax.text(x + 1.5, y + 0.16, f"quick stop: {m['label']}", fontsize=7.5, color=INK,
                    va="bottom")
        elif m["kind"] in ("frame", "late", "actuated"):
            below = m["kind"] == "late"
            ax.text(x, y + (-0.42 if below else 0.2), str(m["label"]), ha="center",
                    fontsize=6.5, color=MUTED)
    for a, b, state in data["states"]:
        name, color = STATES.get(state, (f"0x{state:02X}", GRAY))
        ax.barh(LANES["state"], b - a, left=a, height=0.34, color=color, alpha=0.85, zorder=2)
        if b - a > 12:
            ax.text((a + b) / 2, LANES["state"] + 0.26, name, ha="center", fontsize=7.5, color=INK)

    rules = "; ".join(data["rules"])
    ax.set_title(f"{ROWS[row]}   (proxy rule: {rules})\n{textwrap.fill(STORY[row], 125)}\n"
                 f"recording: tools/interop/recorded/{row}.json.gz, node {NODE}, tag {data['tag']}",
                 fontsize=9, color=INK, loc="left")
    ax.set_yticks(list(LANE_NAMES), [LANE_NAMES[k] for k in LANE_NAMES], fontsize=7.5)
    ax.set_ylim(0.45, 5.6)
    ax.set_xlim(T0, T1)
    ax.tick_params(colors=MUTED, labelsize=8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)


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
    fig.savefig(out, format="svg", facecolor="white", metadata=svg_metadata())
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
