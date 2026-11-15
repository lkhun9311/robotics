#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The E3a bus-load table of README.md, from tools/od/bandwidth.py run on the drive EDS.

Prints the Markdown table; --write-readme replaces the table between the two marker comments in
README.md. tools/evidence/tests/test_evidence.py checks that README.md holds exactly what this
prints, so no number in the table is typed by hand.

  python3 tools/evidence/bandwidth_table.py [--write-readme]
"""
import argparse
import sys

from common import EDS_PATH, ROOT

sys.path.insert(0, str(ROOT / "tools" / "od"))
import bandwidth  # noqa: E402

README = ROOT / "README.md"
BEGIN = "<!-- bandwidth table: python3 tools/evidence/bandwidth_table.py --write-readme -->"
END = "<!-- end of bandwidth table -->"
COMMAND = "python3 tools/od/bandwidth.py config/od/vdrive_arm.eds"


def table(result=None):
    b = result or bandwidth.compute(EDS_PATH)
    k = b["cyclic_breakdown"]
    h = b["headroom"]
    mbit = b["bitrate_bit_s"] / 1e6
    rows = [
        ("Cyclic frames per bus cycle", f"{b['cyclic_frames_per_cycle']} per {b['t_bus_ms']:g} ms: {k['sync']} SYNC + "
         f"{k['nodes']} x ({k['rpdo_per_node']} RPDO + {k['tpdo_per_node']} TPDO); "
         f"{', '.join(b['event_driven_not_counted'])} is event-driven and not counted"),
        ("Worst-case bits of an 8-byte frame", f"{b['frame_bits_8_bytes']}, from `{b['formula']}` (with stuffing)"),
        ("Cyclic load, every frame charged 8 bytes", f"{b['conservative']['bits_per_cycle']} bits per cycle, "
         f"{b['conservative']['occupancy_percent']:.2f} % of {mbit:g} Mbit/s"),
        ("Cyclic load, real DLC (SYNC carries none)", f"{b['actual_dlc']['bits_per_cycle']} bits per cycle, "
         f"{b['actual_dlc']['occupancy_percent']:.2f} %"),
        (f"Plus {h['heartbeats_per_100ms']:g} heartbeats per 100 ms and one SDO request and response per cycle",
         f"{h['bits_per_cycle_with_heartbeat_and_sdo']:g} bits per cycle, "
         f"{h['occupancy_percent_with_heartbeat_and_sdo']:.2f} % ({h['free_percent']:.2f} % free)"),
    ]
    lines = ["| Bus load (computed bound) | Value |", "|---|---|"] + [f"| {a} | {v} |" for a, v in rows]
    return "\n".join(lines) + "\n"


def readme_block(text):
    """The text between BEGIN and END in README.md, without the marker lines."""
    start = text.index(BEGIN) + len(BEGIN) + 1
    return text[start:text.index(END, start)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--write-readme", action="store_true")
    args = parser.parse_args(argv)
    print(f"input: {COMMAND}")
    rendered = table()
    if args.write_readme:
        text = README.read_text(encoding="utf-8")
        old = readme_block(text)
        README.write_text(text.replace(BEGIN + "\n" + old + END, BEGIN + "\n" + rendered + END), encoding="utf-8")
        print(f"wrote the table into {README}")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
