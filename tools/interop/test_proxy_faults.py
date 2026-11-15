# SPDX-License-Identifier: Apache-2.0
"""Proxy faults reach the drive as specified (E3b Task 7).

Topology as the matrix: lely_peer on vcan_host <-> bus_fault_proxy <-> vcan_device <->
vdrive_bank, now with one rule file of config/interop/faults per row. Each fault row runs
pdo_stream --node 3 --count 60, reads drive_flags (0x2030:05) by one SDO upload on vcan_host
after the stream, stops everything and hands the observation to its judge in judges.py
(F1-F6). The restart row (R1) replays frames of a killed bank's tag into its successor; the
cost row publishes the proxy's normal-path cost (a measurement, no threshold).
"""
import json
import os
import shutil
import subprocess
import sys
import time
import zlib

import judges
from conftest import DEVICE_IF, HOST_IF, ROOT, inconclusive
from test_matrix import PEER_DEADLINE_S, judge, observe

sys.path.insert(0, str(ROOT / "tools" / "bench"))
import proxy_cost  # noqa: E402

NODE = judges.NODE
FAULTS = ROOT / "config" / "interop" / "faults"
ARTIFACTS = ROOT / "artifacts"
COST_FRAMES = 5000  # B10 sample size; fewer is INCONCLUSIVE.
COST_LIMIT_S = 60
CONTROL_SEQ = 1000  # R1: the control fragment's seq, one no stream reaches.
# F7 and F2 lose single updates: their streams are valid for the drive's whole lease, so the
# previous command outlasts a lost update by 8 ms (judges.GAP_VALIDITY_MIN_US).
GAP_VALIDITY_MS = 48


def rules_text(name):
    return (FAULTS / name).read_text()


def wait_done(peer, scenario):
    peer.wait_for(lambda e: e.get("event") in ("scenario_done", "scenario_failed"),
                  PEER_DEADLINE_S + 5, f"the end of {scenario}")


def read_drive_flags(raw, harness):
    """One SDO upload of 0x2030:05 on vcan_host once the master has gone; the judge reads the
    response from the capture."""
    time.sleep(0.05)
    harness["t_flags"] = raw.send(*judges.cia301.sdo_upload_request(NODE, *judges.DRIVE_FLAGS))
    time.sleep(0.1)


def stream_through(row, rules, capture, start_bank, start_proxy, start_peer, peer,
                   count=judges.STREAM_COUNT, **peer_args):
    """pdo_stream through `rules`; returns the observation."""
    host, device = capture(HOST_IF), capture(DEVICE_IF)
    proxy = start_proxy(rules)
    bank = start_bank()
    bank.trace.wait_for(lambda r: r.get("event") == "start", 2, "bank start record")
    lely = start_peer("pdo_stream", node=NODE, count=count, **peer_args)
    harness = {"rules": rules, "count": count}
    wait_done(lely, "pdo_stream")
    lely.wait_exit(timeout=5)
    read_drive_flags(peer(HOST_IF), harness)
    return observe(row, host, device, lely, bank, proxy, harness)


def test_late_rpdo2_is_dropped_not_valid(capture, start_bank, start_proxy, start_peer, peer):
    judge(stream_through("F7", rules_text("delay_some_rpdo2.rules"), capture, start_bank,
                         start_proxy, start_peer, peer, validity_ms=GAP_VALIDITY_MS))


def test_all_late_rpdo2_times_out_to_quick_stop(capture, start_bank, start_proxy, start_peer,
                                                peer):
    judge(stream_through("F1", rules_text("delay_rpdo2.rules"), capture, start_bank,
                         start_proxy, start_peer, peer))


def test_drop_every_third_rpdo1_is_an_ordinary_gap(capture, start_bank, start_proxy, start_peer,
                                                    peer):
    judge(stream_through("F2", rules_text("drop_every_third_rpdo1.rules"), capture, start_bank,
                         start_proxy, start_peer, peer, validity_ms=GAP_VALIDITY_MS))


def test_duplicate_fragments_change_nothing(capture, start_bank, start_proxy, start_peer, peer):
    baseline = stream_through("F3-baseline", "", capture, start_bank, start_proxy, start_peer,
                              peer)
    obs = stream_through("F3", rules_text("dup_rpdo1_rpdo2.rules"), capture, start_bank,
                         start_proxy, start_peer, peer)
    obs["baseline"] = baseline
    judge(obs)


def test_reorder_within_window_still_pairs(capture, start_bank, start_proxy, start_peer, peer):
    judge(stream_through("F4", rules_text("reorder_rpdo1_rpdo2.rules"), capture, start_bank,
                         start_proxy, start_peer, peer))


def test_conflicting_copy_is_an_integrity_stop(capture, start_bank, start_proxy, start_peer,
                                               peer):
    judge(stream_through("F5", rules_text("conflicting_rpdo1.rules"), capture, start_bank,
                         start_proxy, start_peer, peer))


def test_d2h_drop_hides_feedback_not_drive_state(capture, start_bank, start_proxy, start_peer,
                                                 peer):
    # Lely never sees a TPDO1, so the scenario reads the statusword by SDO (--statusword sdo).
    judge(stream_through("F6", rules_text("drop_tpdo1.rules"), capture, start_bank, start_proxy,
                         start_peer, peer, statusword="sdo"))


def sdo_result(console, line, index, sub):
    """Runs one console SDO line; returns the value of its sdo_ok, INCONCLUSIVE otherwise."""
    t = console.command(line)["t"]
    results = [e for e in console.events if e.get("event", "").startswith("sdo_") and
               e.get("index") == index and e.get("sub") == sub and e["t"] <= t]
    if not results or results[-1]["event"] != "sdo_ok":
        inconclusive(f"{line!r}: {results[-1:]}")
    return results[-1]["value"]


def wait_statusword(console, state, after):
    console.wait_for(lambda e: e.get("event") == "rpdo" and e.get("index") == 0x6041 and
                     e["value"] & 0x6F == state and e["t"] > after, 2,
                     f"statusword state 0x{state:02X}")


def commit_tag(console):
    """NMT Start, controlwords 0x06 and 0x07, the commit transaction (D17) through Lely's SDO
    client; returns the tag read back from 0x2012. The drive stays in Switched On: no 0x0F, so
    no guard runs and the tag stays current while the bus is quiet."""
    t = console.command(f"nmt {NODE} 1")["t"]
    console.wait_for(lambda e: e.get("event") == "state" and e.get("node") == NODE and
                     e.get("state") == 5 and e["t"] > t, 2, "Lely state Operational")
    wait_statusword(console, 0x21, console.command(f"cw {NODE} 0x06")["t"])
    wait_statusword(console, 0x23, console.command(f"cw {NODE} 0x07")["t"])
    attempt = sdo_result(console, f"read {NODE} 0x2016 0 u32", 0x2016, 0) + 1
    words = (attempt, attempt, 0, 1)  # attempt_id, generation_lo, generation_hi, session_lo
    for sub, value in ((4, words[0]), (1, words[1]), (2, words[2]), (3, words[3])):
        sdo_result(console, f"write {NODE} 0x2010 {sub} u32 {value}", 0x2010, sub)
    crc = zlib.crc32(b"".join(w.to_bytes(4, "little") for w in words))
    sdo_result(console, f"write {NODE} 0x2011 0 u32 {crc}", 0x2011, 0)
    tag = sdo_result(console, f"read {NODE} 0x2012 0 u16", 0x2012, 0)
    if sdo_result(console, f"read {NODE} 0x2016 0 u32", 0x2016, 0) != attempt or tag == 0:
        inconclusive("the commit transaction did not commit")
    return tag


def test_restart_rejects_captured_old_tag_frames(capture, start_bank, start_proxy, start_peer,
                                                 peer):
    host, device = capture(HOST_IF), capture(DEVICE_IF)
    proxy = start_proxy()
    harness = {"count_first": 20, "count_second": 10, "collisions": [], "bank_killed": []}
    peers, banks = [], []

    def bank_started():
        bank = start_bank()
        bank.trace.wait_for(lambda r: r.get("event") == "start", 2, "bank start record")
        banks.append(bank)
        return bank

    bank = bank_started()
    first = start_peer("pdo_stream", node=NODE, count=harness["count_first"])
    peers.append(first)
    wait_done(first, "pdo_stream")
    first.wait_exit(timeout=5)
    done = first.of("scenario_done")
    if not done:
        inconclusive(f"the first stream failed: {first.of('scenario_failed')}")
    old_tag = done[0]["tag"]
    time.sleep(0.05)
    cobs = (judges.cia301.RPDO_BASE[1] + NODE, judges.cia301.RPDO_BASE[2] + NODE)
    old = [(c, d) for _, c, d in list(device.frames)
           if c in cobs and len(d) == 8 and d[6:8] == old_tag.to_bytes(2, "little")]
    harness["bank_killed"].append(bank.kill())
    for _ in range(2):
        bank = bank_started()
        console = start_peer("console", node=NODE)
        peers.append(console)
        console.wait_for(lambda e: e.get("event") == "boot" and e.get("node") == NODE, 5,
                         f"Lely's boot of node {NODE}")
        time.sleep(0.15)
        new_tag = commit_tag(console)
        if new_tag != old_tag:
            break
        # The D3 residual (about 2^-16): the restarted node drew the old tag again. Recorded,
        # and the bank is restarted once more.
        harness["collisions"].append({"tag": new_tag, "t": time.monotonic()})
        console.send("quit")
        console.wait_exit(timeout=5)
        harness["bank_killed"].append(bank.kill())
    else:
        inconclusive(f"the restarted drive committed the old tag {old_tag} twice")
    harness["new_tag"] = new_tag
    console.send("quit")
    console.wait_exit(timeout=5)
    # The bus is quiet now (no master, so no SYNC): the old frames, then one SYNC.
    raw = peer(DEVICE_IF)
    harness["replay"] = []
    for c, d in old:
        harness["replay"].append([raw.send(c, d), c, d.hex()])
        time.sleep(0.001)
    # Control: one RPDO1 of the new tag (a seq the drive has not seen). While the tag is
    # current it is held in the window and dropped as a lone half at the SYNC; were the tag
    # revoked, it would be one more other-tag drop and the replay would prove nothing.
    control = judges.encode("RPDO1", {"target_position": 0, "command_seq": CONTROL_SEQ,
                                      "exec_tag": new_tag})
    harness["control"] = [raw.send(judges.cia301.RPDO_BASE[1] + NODE, control),
                          judges.cia301.RPDO_BASE[1] + NODE, control.hex()]
    time.sleep(0.02)
    harness["t_sync"] = raw.send(judges.cia301.SYNC_COB)
    time.sleep(0.05)
    harness["t_second_stream"] = time.monotonic()
    second = start_peer("pdo_stream", node=NODE, count=harness["count_second"])
    peers.append(second)
    wait_done(second, "pdo_stream")
    second.wait_exit(timeout=5)
    bank.stop()
    obs = observe("R1", host, device, second, bank, proxy, harness)
    obs.pop("peer")
    obs.pop("bank")
    obs["peers"] = [[{k: v for k, v in e.items() if k != "t_rx"} for e in p.events]
                    for p in peers]
    obs["banks"] = [b.trace.poll() for b in banks]
    judge(obs)


def commit_sha():
    sha = os.environ.get("GITHUB_SHA")
    if sha:
        return sha
    try:
        out = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True,
                             text=True, timeout=5)
    except OSError:
        return None
    return out.stdout.strip() if out.returncode == 0 else None


def test_proxy_cost_recorded(start_bank, start_proxy, start_peer):
    """B10: the proxy's normal-path cost over at least 5000 forwarded frames (the Lely master's
    SYNC and PDOs, the bank's heartbeats), published to artifacts/proxy-cost.json. A
    measurement: only the sample size and the shape of the summary are asserted."""
    proxy = start_proxy()
    bank = start_bank()
    bank.trace.wait_for(lambda r: r.get("event") == "start", 2, "bank start record")
    lely = start_peer("idle")
    deadline = time.monotonic() + COST_LIMIT_S
    forwarded = 0
    while forwarded < COST_FRAMES and time.monotonic() < deadline:
        time.sleep(0.2)
        io = proxy.trace.of("io")
        forwarded = sum(io[-1]["forwarded"].values()) if io else 0
    lely.terminate()
    bank.stop()
    proxy.stop()
    summary = proxy_cost.summarize(proxy.trace.path)
    if summary["count"] < COST_FRAMES:
        inconclusive(f"only {summary['count']} normal-path frames in {COST_LIMIT_S} s")
    assert summary["samples"] == summary["count"], summary
    assert 0 < summary["p50"] <= summary["p99"] <= summary["max"], summary
    ARTIFACTS.mkdir(exist_ok=True)
    shutil.copy(proxy.trace.path, ARTIFACTS / "proxy-cost-trace.jsonl")
    out = ARTIFACTS / "proxy-cost.json"
    assert proxy_cost.main([str(proxy.trace.path), "--out", str(out),
                            "--commit", commit_sha() or "unknown"]) == 0
    written = json.loads(out.read_text())
    print(f"proxy cost: {json.dumps({k: v for k, v in written.items() if k != 'dist'})}")
