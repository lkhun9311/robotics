# SPDX-License-Identifier: Apache-2.0
"""The vdrive_bank and bus_fault_proxy processes on vcan (E3b Task 4).

Every test first asserts its preconditions (process ready, the declared traffic on the wire, a
complete trace); a missing one fails with INCONCLUSIVE. These assert protocol facts, ordering and
liveness; interval distributions are printed as measurements, not gated (shared runners).
"""
import os
import re
import shutil
import statistics
import subprocess
import sys
import time

from conftest import (BUS_FAULT_PROXY_BIN, DEVICE_IF, HOST_IF, NODES, ROOT, VCAN_UP, inconclusive)

BOOT_UP = 0x00
PRE_OPERATIONAL = 0x7F
CAN_RTR_FLAG = 0x40000000


def wait_frames(cap, predicate, count, timeout):
    """The first `count` captured frames satisfying predicate, waiting up to timeout seconds."""
    deadline = time.monotonic() + timeout
    while True:
        found = [f for f in list(cap.frames) if predicate(f)]
        if len(found) >= count or time.monotonic() >= deadline:
            return found
        time.sleep(0.01)


def sdo_upload(index, sub=0):
    return bytes([0x40, index & 0xFF, index >> 8, sub, 0, 0, 0, 0])


def test_bank_boots_six_nodes(capture, start_bank):
    cap = capture(DEVICE_IF)
    t_start = time.monotonic()
    bank = start_bank()
    start = bank.trace.wait_for(lambda r: r.get("event") == "start", 2, "start record")
    boot_ids = start["boot_ids"]
    assert len(boot_ids) == 6 and all(boot_ids), boot_ids
    assert len(set(boot_ids)) == 6, f"boot_ids not distinct: {boot_ids}"

    def is_boot(f):
        return 0x701 <= f[1] <= 0x706 and f[2] == bytes([BOOT_UP])
    boots = wait_frames(cap, is_boot, 6, 2.0)
    late = [f for f in boots if f[0] - t_start > 2.0]
    assert sorted(f[1] - 0x700 for f in boots) == list(NODES), boots
    assert not late, f"boot-up later than 2 s after start: {late}"


def test_heartbeat_present_on_the_wire(capture, bank):
    cap = capture(DEVICE_IF)
    time.sleep(1.5)
    frames = cap.stop()
    for node in NODES:
        beats = [(t, d) for t, cob, d in frames if cob == 0x700 + node]
        times = [t for t, _ in beats]
        gaps = [round((b - a) * 1000, 2) for a, b in zip(times, times[1:])]
        if gaps:
            print(f"node {node}: {len(beats)} heartbeats in 1.5 s, interval ms "
                  f"min {min(gaps)} median {statistics.median(gaps)} max {max(gaps)}")
        assert len(beats) >= 10, f"node {node}: {len(beats)} heartbeats in 1.5 s"
        states = {d for _, d in beats}
        assert states == {bytes([PRE_OPERATIONAL])}, f"node {node}: state bytes {states}"


def test_proxy_forwards_both_ways_without_circulation(capture, proxy, peer):
    host_cap, device_cap = capture(HOST_IF), capture(DEVICE_IF)
    host, device = peer(HOST_IF), peer(DEVICE_IF)
    down, up = (0x123, b"\x01\x02\x03"), (0x456, b"\x0a")
    t_down = host.send(*down)
    t_up = device.send(*up)
    seen_down = wait_frames(device_cap, lambda f: (f[1], f[2]) == down, 1, 0.2)
    seen_up = wait_frames(host_cap, lambda f: (f[1], f[2]) == up, 2, 0.2)
    time.sleep(0.2)  # Anything circulating would arrive within this.
    host_frames, device_frames = host_cap.stop(), device_cap.stop()

    def count(frames, frame):
        return [f[0] for f in frames if (f[1], f[2]) == frame]
    # Host side: the original down frame once (not echoed back) and the forwarded up frame once.
    assert len(count(host_frames, down)) == 1, host_frames
    assert len(count(host_frames, up)) == 1, host_frames
    assert len(count(device_frames, down)) == 1, device_frames
    assert len(count(device_frames, up)) == 1, device_frames
    assert seen_down and seen_down[0][0] - t_down <= 0.2
    assert seen_up and seen_up[0][0] - t_up <= 0.2
    exit_record = proxy.stop()
    assert exit_record["forwarded"] == {"h2d": 1, "d2h": 1}, exit_record
    assert exit_record["dropped_at_exit"] == 0
    print(f"proxy cost (ns, measurement): {exit_record['cost_ns']}")


def test_proxy_rejects_bad_rules(vcan, tmp_path):
    rules = tmp_path / "bad.rules"
    rules.write_text("# drop SYNC\nh2d 0x080/0x7FF drop\nh2d 0x080/0x7FF explode\n")
    assert BUS_FAULT_PROXY_BIN.exists(), f"NOT_RUN: {BUS_FAULT_PROXY_BIN} not built"
    result = subprocess.run([str(BUS_FAULT_PROXY_BIN), "--host", HOST_IF, "--device", DEVICE_IF,
                             "--rules", str(rules), "--trace", str(tmp_path / "t.jsonl")],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 2, result
    assert "line 3" in result.stderr, result.stderr
    assert not (tmp_path / "t.jsonl").exists(), "a rejected start must not begin a trace"


def test_trace_reports_io_counters(start_bank, peer, capture):
    cap = capture(DEVICE_IF)
    bank = start_bank()
    sender = peer(DEVICE_IF)
    for _ in range(10):
        sender.send(0x601, sdo_upload(0x1000))
    for _ in range(3):
        sender.send(0x602, sdo_upload(0x1017))
    sender.send(0x601, flags=CAN_RTR_FLAG)  # Received and counted, never dispatched (B11).
    responses = wait_frames(cap, lambda f: f[1] in (0x581, 0x582), 13, 2.0)
    if len(responses) != 13:
        inconclusive(f"expected 13 SDO responses on the wire, saw {len(responses)}")
    time.sleep(1.1)  # At least one periodic snapshot as well.
    exit_record = bank.stop()
    records = bank.trace.records
    periodic = [r for r in records[:-8] if r["event"] == "io"]
    assert periodic, "no periodic io snapshot within 1.1 s"

    final = records[-9:-1]  # loop, dispatch x6, io, then exit.
    assert [r["event"] for r in final] == ["loop"] + ["dispatch"] * 6 + ["io"], final
    dispatch = {r["node"]: r for r in final if r["event"] == "dispatch"}
    assert sorted(dispatch) == list(NODES)
    for node, r in dispatch.items():
        for key in ("nmt", "heartbeat", "sync", "emcy", "sdo", "pdo", "gated", "foreign",
                    "malformed"):
            assert key in r, (node, key, r)
        assert r["sdo"] == {1: 10, 2: 3}.get(node, 0), (node, r)
        assert r["trace_overflow"] == 0
    io = final[-1]
    assert io["rx"] >= 14, io
    assert io["rx_rejected"]["rtr"] == 1, io
    assert io["tx"] >= 6 + 13, io
    assert io["tx_eagain"] == io["tx_enobufs"] == io["tx_other_error"] == 0, io
    loop = final[0]
    assert loop["releases_executed"] > 0 and loop["frames_dispatched"] >= 13, loop
    print(f"loop counters: {loop}")
    assert exit_record["signal"] == 15


def test_vcan_unavailable_is_not_run(tmp_path):
    forced = subprocess.run([str(VCAN_UP)], capture_output=True, text=True,
                            env={**os.environ, "VCAN_UP_FORCE_FAIL": "1"})
    assert forced.returncode == 3, forced
    assert forced.stdout.startswith("NOT_RUN: "), forced.stdout

    # Without NET_ADMIN (dropped from the bounding set, so the shell cannot regain it).
    setpriv = shutil.which("setpriv")
    assert setpriv, "INCONCLUSIVE: setpriv (util-linux) not found"
    no_cap = subprocess.run([setpriv, "--inh-caps=-net_admin", "--bounding-set=-net_admin",
                             str(VCAN_UP)], capture_output=True, text=True)
    assert no_cap.returncode == 3, no_cap
    assert no_cap.stdout.startswith("NOT_RUN: "), no_cap.stdout

    # The fixture turns exit 3 into a failure carrying NOT_RUN, never a skip.
    nested = subprocess.run(
        [sys.executable, "-m", "pytest", "-p", "no:cacheprovider", "-p", "no:launch_testing",
         "-p", "no:launch_ros", "-q", "-rA",
         f"{__file__}::test_bank_boots_six_nodes"],
        capture_output=True, text=True, cwd=ROOT, env={**os.environ, "VCAN_UP_FORCE_FAIL": "1"})
    assert nested.returncode == 1, nested.stdout[-2000:]
    assert "NOT_RUN: forced by VCAN_UP_FORCE_FAIL=1" in nested.stdout, nested.stdout[-2000:]
    # A fixture failure is reported as an error; either way it is red, never skipped.
    assert re.search(r"\b1 (failed|error)\b", nested.stdout), nested.stdout[-2000:]
    assert "skipped" not in nested.stdout, nested.stdout[-2000:]


def test_proxy_reports_a_dead_interface_without_spinning(proxy):
    """A socket error (the interface goes down) is read, counted and reported; the proxy exits
    non-zero with the reason instead of polling the pending error forever."""
    t_down = time.monotonic()
    down = subprocess.run(["ip", "link", "set", "dev", DEVICE_IF, "down"], capture_output=True,
                          text=True)
    try:
        assert down.returncode == 0, f"INCONCLUSIVE: cannot take {DEVICE_IF} down: {down.stderr}"
        rc = proxy.wait_exit(2.0)
        elapsed = time.monotonic() - t_down
    finally:
        up = subprocess.run(["ip", "link", "set", "dev", DEVICE_IF, "up"], capture_output=True,
                            text=True)
        assert up.returncode == 0, up.stderr
    assert rc != 0, "the proxy kept running with its device interface down"
    records = proxy.trace.records
    errors = [r for r in records if r["event"] == "error"]
    assert errors and DEVICE_IF in errors[-1]["reason"], records[-3:]
    snapshots = [r for r in records if r["event"] == "io"]
    # One snapshot per second plus the final one: a spinning loop would not change this count, but
    # the counted receive error shows the error was read, not polled forever.
    assert len(snapshots) <= 3, f"{len(snapshots)} snapshots in {elapsed:.2f} s"
    assert snapshots and snapshots[-1]["device"]["rx_error"] >= 1, snapshots[-1:]
    print(f"proxy ended {elapsed * 1000:.0f} ms after the link went down: {errors[-1]}")
