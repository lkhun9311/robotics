# SPDX-License-Identifier: Apache-2.0
"""Shared helpers of the E3b evidence scripts: recorded observations, PDO decoding through the
EDS layout of tools/od/wire.py, and the commands that reproduce each input.

The inputs are the observations the interop tests record (tools/interop/recorded/*.json.gz,
written with INTEROP_RECORD_DIR) and the proxy cost JSON written by tools/bench/proxy_cost.py.
Nothing here judges: the verdicts stay with tools/interop/judges.py. Frames and PDO fields are
parsed by tools/interop/observation.py, the same code the judges read them with.
"""
import gzip
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "od"))
sys.path.insert(0, str(ROOT / "tools" / "interop"))

import observation  # noqa: E402
import wire  # noqa: E402

RECORDED = ROOT / "tools" / "interop" / "recorded"
# Our master against CANopenNode (E3c Task 8, rows n1-n7) is recorded in a directory of its own.
RECORDED_MASTER = RECORDED / "master"
MEDIA = ROOT / "docs" / "media"
EDS_PATH = ROOT / "config" / "od" / "vdrive_arm.eds"
NODE = 3  # The node the matrix and the fault rows drive.

# The pytest test that writes each recording, by row.
_M = "tools/interop/test_matrix.py::"
_F = "tools/interop/test_proxy_faults.py::"
_X = "tools/interop/test_master_faults.py::"
_N = "tools/interop/test_master_matrix.py::"
RECORDING_TESTS = {
    "m1": _M + "test_nmt_states_and_service_gating",
    "m2": _M + "test_first_heartbeat_missing_vs_running_timeout",
    "m3": _M + "test_sdo_expedited_and_aborts",
    "m4": _M + "test_pdo_bytes_and_lengths_seen_by_peer",
    "m5": _M + "test_sync_application_and_sampling_order",
    "m6": _M + "test_emcy_production_and_reception",
    "f1": _F + "test_all_late_rpdo2_times_out_to_quick_stop",
    "f2": _F + "test_drop_every_third_rpdo1_is_an_ordinary_gap",
    "f3": _F + "test_duplicate_fragments_change_nothing",
    "f4": _F + "test_reorder_within_window_still_pairs",
    "f5": _F + "test_conflicting_copy_is_an_integrity_stop",
    "f6": _F + "test_d2h_drop_hides_feedback_not_drive_state",
    "f7": _F + "test_late_rpdo2_is_dropped_not_valid",
    "r1": _F + "test_restart_rejects_captured_old_tag_frames",
    "x1": _X + "test_short_feedback_outage_latches_inhibit",
    "x2": _X + "test_dropped_tpdos_make_feedback_stale",
    "x3": _X + "test_one_node_command_loss_stops_the_group",
    "x4": _X + "test_heartbeat_loss_raises_fault_without_reset",
    "x5": _X + "test_sdo_timeout_during_arm_fails_arming",
    "x6": _X + "test_all_commands_delayed_stop_the_group",
    "x7": _X + "test_minority_delayed_commands_never_actuate_late",
    "n1": _N + "test_master_nmt_states_and_service_gating",
    "n2": _N + "test_master_first_heartbeat_missing_vs_running_timeout",
    "n3": _N + "test_master_sdo_expedited_and_aborts",
    "n4": _N + "test_master_pdo_bytes_and_lengths",
    "n5": _N + "test_master_sync_application_and_sampling_order",
    "n6": _N + "test_master_emcy_reception",
    "n7": _N + "test_master_refuses_activation_on_canopennode",
}


def recorded_dir(row):
    """Where the committed recording of `row` lives."""
    return RECORDED_MASTER if row.startswith("n") else RECORDED
PYTEST = "python3 -m pytest -p no:cacheprovider -p no:launch_testing -p no:launch_ros -q"
# The vcan tests need the workspace built (run_checks.sh does it) and the links up.
VCAN_PREFIX = ("docker compose run --rm -T dev bash -c 'tools/bus/vcan_up.sh && "
               "source install/release/setup.bash && ")


def recording_command(row):
    """The command that records `row` again (a new run: new times and values)."""
    directory = recorded_dir(row).relative_to(ROOT)
    return (f"{VCAN_PREFIX}INTEROP_RECORD_DIR={directory} {PYTEST} "
            f"{RECORDING_TESTS[row]}'")


PROXY_COST_COMMAND = (
    f"{VCAN_PREFIX}{PYTEST} tools/interop/test_proxy_faults.py::test_proxy_cost_recorded' && "
    "python3 tools/bench/proxy_cost.py artifacts/proxy-cost-trace.jsonl "
    "--out docs/media/e3b-proxy-cost.json --commit $(git rev-parse HEAD) --host <host>")


def print_input(what, command):
    """Every script names the command that reproduces its input."""
    print(f"input: {what}")
    print(f"reproduce the input: {command}")


def load_recording(row, directory=RECORDED):
    with gzip.open(pathlib.Path(directory) / f"{row}.json.gz", "rt") as f:
        return json.load(f)


_LAYOUTS = None


def layouts():
    """{COB-ID of node 3: PDO layout} from the EDS, through wire.py."""
    global _LAYOUTS
    if _LAYOUTS is None:
        _LAYOUTS = {}
        for pdo in wire.Eds(EDS_PATH).pdos():
            _LAYOUTS[wire.pdo_cob_id(pdo, NODE)] = pdo
    return _LAYOUTS


def layout(label):
    return next(p for p in layouts().values() if p["label"] == label)


def cob(label, node=NODE):
    return wire.pdo_cob_id(layout(label), node)


def decode(label, data):
    """Field values of one PDO frame by its EDS mapping. The bytes are checked by re-encoding
    them with wire.pdo_frame, so a decode that disagrees with the encoder raises."""
    pdo = layout(label)
    if len(data) != pdo["length"]:
        raise ValueError(f"{label} frame of {len(data)} bytes, mapping is {pdo['length']}")
    values = observation.pdo_fields(pdo, data)
    if bytes(wire.pdo_frame(pdo, NODE, values, "check")["data"]) != bytes(data):
        raise ValueError(f"{label} {data.hex()} does not re-encode")
    return values


def frames(obs, side, cobs=None, t0=None, t1=None):
    """[(t s, COB-ID, bytes)] on `side` ('host' or 'device'), t0 <= t <= t1."""
    return observation.frames(obs, side, cobs, t0, t1, inclusive=True)


def drive_records(obs, node=NODE):
    return observation.drive_records(obs, node)


def stream_tag(obs):
    done = [e for e in obs["peer"] if e.get("event") == "scenario_done"]
    if len(done) != 1 or not done[0].get("tag"):
        raise ValueError("the recording has no single pdo_stream scenario_done with a tag")
    return done[0]["tag"]


def rpdo_of_tag(obs, side, label, tag):
    """[(t, seq, bytes)] of node 3's RPDO1 or RPDO2 carrying `tag` on `side`."""
    out = []
    for t, _, d in frames(obs, side, {cob(label)}):
        if len(d) == layout(label)["length"]:
            v = decode(label, d)
            if v["exec_tag"] == tag:
                out.append((t, v["command_seq"], d))
    return out


TOOLCHAIN_PREFIX = "toolchain: "


def toolchain():
    """The matplotlib version and the font file it draws with (name-table version and SHA-256):
    the SVG bytes embed glyph outlines, so either one changes the bytes."""
    import hashlib

    import matplotlib
    from matplotlib import font_manager
    from matplotlib.ft2font import FT2Font

    path = font_manager.findfont("DejaVu Sans")
    names = FT2Font(path).get_sfnt()
    raw = names.get((3, 1, 1033, 5)) or names.get((1, 0, 0, 5)) or b""
    version = raw.decode("utf-16-be" if (3, 1, 1033, 5) in names else "latin-1")
    with open(path, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()[:16]
    return f"matplotlib {matplotlib.__version__}; DejaVu Sans {version} sha256:{digest}"


def svg_metadata():
    """SVG metadata: no date (reproducible bytes) and the toolchain that drew the image."""
    return {"Date": None, "Description": TOOLCHAIN_PREFIX + toolchain()}


def recorded_toolchain(svg_path):
    """The toolchain written into a committed SVG's metadata, or None."""
    import re

    text = pathlib.Path(svg_path).read_text(encoding="utf-8")
    found = re.search(r"<dc:description>" + re.escape(TOOLCHAIN_PREFIX) + r"(.*?)</dc:description>",
                      text)
    return found.group(1) if found else None
