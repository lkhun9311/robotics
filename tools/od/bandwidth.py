# SPDX-License-Identifier: Apache-2.0
"""CAN bandwidth of the E3a wire contract, computed from the drive EDS (D15).

Per bus cycle T_bus (10 ms) at 1 Mbit/s: one SYNC, and for each of 6 drive nodes the cyclic
RPDOs (transmission type 1..240) and TPDOs found in the EDS. RPDO3 (transmission type 255) is
event-driven and is listed but not counted in the cyclic load.

Worst-case bits of one standard (11-bit ID) data frame with n data bytes, including the 3-bit
interframe space and the worst-case stuffing bound (stuffing covers the first 34 + 8n bits):

    bits(n) = 47 + 8*n + floor((34 + 8*n - 1) / 4)      # 135 for n = 8

Two views are reported. `conservative` charges every frame as 8 data bytes (the figure of the
contract). `actual_dlc` uses the mapped length of each PDO (8 bytes for all cyclic PDOs today)
and DLC 0 for SYNC. The test bound applies to `conservative`.
"""
import argparse
import json
import pathlib
import sys

import od

BITRATE = 1_000_000
T_BUS_S = 0.010
NODES = range(1, 7)
HEARTBEAT_PERIOD_S = 0.100
CYCLIC_MAX_TRANSMISSION_TYPE = 240


def frame_bits(n: int) -> int:
    """Worst-case bits of a standard data frame with n data bytes (formula in the module doc)."""
    return 47 + 8 * n + (34 + 8 * n - 1) // 4


def _cob(entry_value, node: int) -> int:
    return entry_value.value + (node if entry_value.node_relative else 0)


def compute(path) -> dict:
    dic = od.load(path)
    problems = od.validate(dic)
    if problems:
        raise ValueError(f"EDS invalid: {problems}")
    sync_cob = dic.entry(0x1005, 0).default().value & 0x7FF
    hb_period_ms = dic.entry(0x1017, 0).default().value
    bits_per_cycle_s = BITRATE * T_BUS_S

    frames = [{"name": "SYNC", "node": 0, "cob_id": sync_cob, "dlc": 0, "cyclic": True}]
    for p in dic.pdos():
        nbytes = p.bits // 8
        cyclic = p.transmission_type is not None and p.transmission_type <= CYCLIC_MAX_TRANSMISSION_TYPE
        for node in NODES:
            frames.append({"name": p.label, "node": node, "cob_id": _cob(p.cob_id, node),
                           "dlc": nbytes, "cyclic": cyclic})
    for f in frames:
        f["bits_conservative"] = frame_bits(8)
        f["bits_actual"] = frame_bits(f["dlc"])

    cyc = [f for f in frames if f["cyclic"]]
    cons = sum(f["bits_conservative"] for f in cyc)
    act = sum(f["bits_actual"] for f in cyc)
    hb_bits = frame_bits(1)
    hb_per_cycle = len(NODES) * T_BUS_S / (hb_period_ms / 1000.0)
    sdo_pair = 2 * frame_bits(8)
    extra = hb_per_cycle * hb_bits + sdo_pair

    def pct(bits):
        return 100.0 * bits / bits_per_cycle_s

    order = sorted(frames, key=lambda f: f["cob_id"])
    return {
        "bitrate_bit_s": BITRATE,
        "t_bus_ms": T_BUS_S * 1000,
        "formula": "bits(n) = 47 + 8*n + floor((34 + 8*n - 1)/4)",
        "frame_bits_8_bytes": frame_bits(8),
        "cyclic_frames_per_cycle": len(cyc),
        "cyclic_breakdown": {"sync": 1, "rpdo_per_node": sum(1 for p in dic.pdos() if p.kind == "rpdo" and p.transmission_type <= CYCLIC_MAX_TRANSMISSION_TYPE),
                             "tpdo_per_node": sum(1 for p in dic.pdos() if p.kind == "tpdo" and p.transmission_type <= CYCLIC_MAX_TRANSMISSION_TYPE),
                             "nodes": len(NODES)},
        "event_driven_not_counted": sorted({f["name"] for f in frames if not f["cyclic"]}),
        "conservative": {"bits_per_cycle": cons, "occupancy_percent": round(pct(cons), 2)},
        "actual_dlc": {"bits_per_cycle": act, "occupancy_percent": round(pct(act), 2)},
        "headroom": {
            "heartbeats_per_100ms": round(len(NODES) * 0.100 / (hb_period_ms / 1000.0), 3),
            "heartbeat_frame_bits": hb_bits,
            "sdo_pair_bits": sdo_pair,
            "bits_per_cycle_with_heartbeat_and_sdo": round(cons + extra, 1),
            "occupancy_percent_with_heartbeat_and_sdo": round(pct(cons + extra), 2),
            "free_percent": round(100 - pct(cons + extra), 2),
        },
        "priority_order": [f"0x{f['cob_id']:03X} {f['name']}" + (f" node {f['node']}" if f["node"] else "")
                           for f in order if f["cyclic"]] + [
            f"0x{f['cob_id']:03X} {f['name']} node {f['node']} (event-driven)" for f in order if not f["cyclic"]],
        "note": "vcan does not model physical arbitration delay or bit stuffing; this is a computed bound.",
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("eds", type=pathlib.Path)
    ap.add_argument("--out", type=pathlib.Path)
    args = ap.parse_args(argv)
    result = compute(args.eds)
    text = json.dumps(result, indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text)
    print(text, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
