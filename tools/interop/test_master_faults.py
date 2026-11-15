# SPDX-License-Identifier: Apache-2.0
"""Master-side faults (E3c Task 7): our canopen_master_cli drives our vdrive_bank through
bus_fault_proxy, and the proxy injects the fault (dropped TPDOs, a dropped heartbeat, dropped SDO
answers, delayed RPDO2). The verdicts are judged from the wire captures of both interfaces, the
CLI's JSON events and the bank's trace (judges_master.py), on a recorded observation like the
E3b rows.

Every command-loss row runs four phases: the healthy arm (all six armed), a moving baseline (at
least 50 dispatched commands actuated on all six), the fault onset, and the stop observation. The
proxy's windows are relative to its start instant, so the CLI holds its arm until a fixed instant
after it (--arm-at-ns, ARM_AT_MS); the rule files open their window well after the arm and the
baseline, and the judge asserts INCONCLUSIVE if the first matched frame precedes the CLI's
recorded arm-complete time or the window matched nothing.
"""
import gzip
import json
import os
import pathlib
import time

import judges_master
from conftest import DEVICE_IF, HOST_IF, ROOT, inconclusive
from test_master_processes import CLI_BIN, run_to_end, start_cli

FAULTS = ROOT / "config" / "interop" / "faults"
ARM_AT_MS = 1500  # After the proxy's start; the rule files' windows are placed against it.
STREAM_MS = 1800  # The stream's length after the arm completes (about 1525 ms).
SETUP_LIMIT_MS = 300  # The CLI must be running this long before its arm instant.
DROPPED_EVENTS = ("pdo", "sync")  # Thousands of lines no judge reads (the wire has them).


def rules_text(name):
    return (FAULTS / name).read_text()


def run_row(row, rules, capture, start_bank, start_proxy, procs, scenario="stream"):
    """Runs the CLI through the proxy rules to the end of its scenario; returns the observation."""
    host, device = capture(HOST_IF), capture(DEVICE_IF)
    proxy = start_proxy(rules)
    bank = start_bank()
    bank.trace.wait_for(lambda r: r.get("event") == "start", 2, "bank start record")
    start = proxy.trace.of("start")
    if len(start) != 1:
        inconclusive(f"no single proxy start record: {start}")
    epoch_ns = start[0]["t_ns"]
    arm_at_ns = epoch_ns + ARM_AT_MS * 1_000_000
    if time.monotonic_ns() > arm_at_ns - SETUP_LIMIT_MS * 1_000_000:
        inconclusive(f"the rig took {(time.monotonic_ns() - epoch_ns) / 1e6:.0f} ms to start; "
                     f"the arm is held until {ARM_AT_MS} ms")
    args = ["--arm-at-ns", str(arm_at_ns)]
    if scenario == "stream":
        args += ["--duration-ms", str(STREAM_MS)]
    cli = start_cli(procs, scenario, *args)
    rc, end = run_to_end(cli, 25)
    time.sleep(0.1)
    harness = {"rules": rules, "epoch_ns": epoch_ns, "arm_at_ns": arm_at_ns, "cli_rc": rc,
               "scenario": scenario, "stream_ms": STREAM_MS}
    bank.stop()
    proxy_exit = proxy.stop()
    obs = {"row": row,
           "host": [[t, c, d.hex()] for t, c, d in host.stop()],
           "device": [[t, c, d.hex()] for t, c, d in device.stop()],
           "cli": [{k: v for k, v in e.items() if k != "t_rx"} for e in cli.events
                   if e.get("event") not in DROPPED_EVENTS],
           "bank": bank.trace.poll(), "proxy_start": start[0], "proxy_exit": proxy_exit,
           "harness": harness}
    return obs


def judge(obs):
    """Judges the observation; with INTEROP_RECORD_DIR set, records it there first."""
    record_dir = os.environ.get("INTEROP_RECORD_DIR")
    if record_dir:
        path = pathlib.Path(record_dir) / f"{obs['row'].lower()}.json.gz"
        path.parent.mkdir(parents=True, exist_ok=True)
        with gzip.open(path, "wt", encoding="utf-8") as f:
            json.dump(obs, f, separators=(",", ":"))
    status, text = judges_master.verdict(obs)
    print(f"{obs['row']} {status}: {text}")
    if status == "INCONCLUSIVE":
        inconclusive(text)
    assert status == "PASS", f"{obs['row']} {status}: {text}"


def row(name, rules_file, **kw):
    def go(capture, start_bank, start_proxy, procs):
        assert CLI_BIN.exists(), f"NOT_RUN: {CLI_BIN} not built"
        judge(run_row(name, rules_text(rules_file), capture, start_bank, start_proxy, procs,
                      **kw))
    return go


def test_short_feedback_outage_latches_inhibit(capture, start_bank, start_proxy, procs):
    row("X1", "master_tpdo_outage_35ms.rules")(capture, start_bank, start_proxy, procs)


def test_dropped_tpdos_make_feedback_stale(capture, start_bank, start_proxy, procs):
    row("X2", "master_tpdo_outage_100ms.rules")(capture, start_bank, start_proxy, procs)


def test_one_node_command_loss_stops_the_group(capture, start_bank, start_proxy, procs):
    row("X3", "master_drop_rpdo2_node3.rules")(capture, start_bank, start_proxy, procs)


def test_heartbeat_loss_raises_fault_without_reset(capture, start_bank, start_proxy, procs):
    row("X4", "master_drop_heartbeat_node4.rules")(capture, start_bank, start_proxy, procs)


def test_sdo_timeout_during_arm_fails_arming(capture, start_bank, start_proxy, procs):
    row("X5", "master_drop_sdo_answers_node5.rules", scenario="arm")(
        capture, start_bank, start_proxy, procs)


def test_all_commands_delayed_stop_the_group(capture, start_bank, start_proxy, procs):
    row("X6", "master_delay_all_rpdo2.rules")(capture, start_bank, start_proxy, procs)


def test_minority_delayed_commands_never_actuate_late(capture, start_bank, start_proxy, procs):
    row("X7", "master_delay_some_rpdo2_node3.rules")(capture, start_bank, start_proxy, procs)
