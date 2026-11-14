# SPDX-License-Identifier: Apache-2.0
"""Interop matrix M1-M6 (E3b Task 6): the Lely master drives our vdrive_bank (roadmap E3 exit).

Topology: lely_peer on vcan_host <-> bus_fault_proxy (no rules) <-> vcan_device <-> vdrive_bank.
Each test runs its scenario, captures both interfaces, the peer's lines and the complete traces,
and hands that observation to its judge in judges.py. A row passes only on both the wire
observation and Lely's own decoded event; a missing precondition fails INCONCLUSIVE. With
INTEROP_RECORD_DIR set, each observation is also written there (judges are pure, so a recorded
observation can be judged again without vcan, see test_oracle_negative.py).
"""
import gzip
import json
import os
import pathlib
import time

import judges
from conftest import DEVICE_IF, HOST_IF, NODES, inconclusive

NODE = judges.NODE
PEER_DEADLINE_S = 15


def observe(row, host, device, peer, bank, proxy, harness):
    """Stops everything (complete traces first) and returns the observation."""
    if peer.proc.poll() is None:
        peer.wait_exit(timeout=5)
    if harness.get("bank_killed") is None:
        bank.stop()
    proxy_exit = proxy.stop()
    obs = {"row": row,
           "host": [[t, c, d.hex()] for t, c, d in host.stop()],
           "device": [[t, c, d.hex()] for t, c, d in device.stop()],
           "peer": [{k: v for k, v in e.items() if k != "t_rx"} for e in peer.events],
           "bank": bank.trace.poll(), "proxy_exit": proxy_exit, "harness": harness}
    record_dir = os.environ.get("INTEROP_RECORD_DIR")
    if record_dir:
        path = pathlib.Path(record_dir) / f"{row.lower()}.json.gz"
        path.parent.mkdir(parents=True, exist_ok=True)
        with gzip.open(path, "wt", encoding="utf-8") as f:
            json.dump(obs, f, separators=(",", ":"))
    return obs


def judge(obs):
    status, text = judges.verdict(obs)
    print(f"{obs['row']} {status}: {text}")
    if status == "INCONCLUSIVE":
        inconclusive(text)
    assert status == "PASS", f"{obs['row']} {status}: {text}"


def start_rig(capture, start_bank, start_proxy, start_peer, scenario, bank_first=True,
              hb_muted=None, before_peer=None, **peer_args):
    """Captures, the proxy, the bank (with inject) and the peer, in the order the row needs."""
    host, device = capture(HOST_IF), capture(DEVICE_IF)
    proxy = start_proxy()
    harness = {}
    bank = None
    if bank_first:
        bank = start_bank(inject=True)
        bank.trace.wait_for(lambda r: r.get("event") == "start", 2, "bank start record")
        if before_peer is not None:
            before_peer(bank)
    peer = start_peer(scenario, **peer_args)
    if not bank_first:
        time.sleep(0.3)
        harness["t_bank_start"] = time.monotonic()
        bank = start_bank(inject=True, hb_muted=hb_muted)
        bank.trace.wait_for(lambda r: r.get("event") == "start", 2, "bank start record")
    return host, device, proxy, bank, peer, harness


def wait_booted(peer, nodes=NODES, timeout=5):
    for n in nodes:
        peer.wait_for(lambda e, n=n: e.get("event") == "boot" and e.get("node") == n, timeout,
                      f"boot of node {n}")


def wait_state(peer, state, after, timeout=2):
    peer.wait_for(lambda e: e.get("event") == "state" and e.get("node") == NODE and
                  e.get("state") == state and e["t"] > after, timeout, f"Lely state {state}")


def wait_new_boot(peer, after, timeout=3):
    peer.wait_for(lambda e: e.get("event") == "boot" and e.get("node") == NODE and
                  e["t"] > after, timeout, f"a new boot of node {NODE}")


def quit_console(peer):
    peer.send("quit")
    peer.wait_for(lambda e: e.get("event") in ("scenario_done", "scenario_failed"), 5,
                  "the end of the console")


def test_nmt_states_and_service_gating(capture, start_bank, start_proxy, start_peer):
    host, device, proxy, bank, peer, harness = start_rig(
        capture, start_bank, start_proxy, start_peer, "console", node=NODE)
    wait_booted(peer)
    dwell = 0.3
    t = peer.command(f"nmt {NODE} 1")["t"]
    wait_state(peer, 5, t)
    time.sleep(dwell)
    t = peer.command(f"nmt {NODE} 2")["t"]
    wait_state(peer, 4, t)
    time.sleep(dwell)
    peer.command(f"read {NODE} 0x1000 0 u32")  # Ends with Lely's 1 s SDO timeout.
    time.sleep(dwell)
    t = peer.command(f"nmt {NODE} 128")["t"]
    wait_state(peer, 127, t)
    time.sleep(dwell)
    peer.command(f"read {NODE} 0x1000 0 u32")
    time.sleep(dwell)
    t = peer.command(f"nmt {NODE} 130")["t"]
    wait_new_boot(peer, t)
    time.sleep(dwell)
    t = peer.command(f"nmt {NODE} 129")["t"]
    wait_new_boot(peer, t)
    time.sleep(dwell)
    quit_console(peer)
    judge(observe("M1", host, device, peer, bank, proxy, harness))


def test_first_heartbeat_missing_vs_running_timeout(capture, start_bank, start_proxy,
                                                    start_peer, inject_client):
    node = judges.M2_NODE
    host, device, proxy, bank, peer, harness = start_rig(
        capture, start_bank, start_proxy, start_peer, "console", bank_first=False,
        hb_muted=str(node), node=node)
    # Phase A: the boot-up, then no heartbeat; Lely's boot of node 2 ends with 'K'.
    peer.wait_for(lambda e: e.get("event") == "boot" and e.get("node") == node, 5,
                  f"Lely's boot result for node {node}")
    wait_booted(peer)
    for sub in (1, 2, 3, 4):
        peer.command(f"read {node} 0x1018 {sub} u32")
    time.sleep(0.5)
    client = inject_client(bank)
    # Phase B: heartbeats for 0.5 s, then muted again.
    harness["t_unmute_b"] = time.monotonic()
    if client.ask(f"hb_mute {node} 0".encode()) != b"ok":
        inconclusive("the bank refused the unmute")
    time.sleep(0.5)
    harness["t_mute_b"] = time.monotonic()
    if client.ask(f"hb_mute {node} 1".encode()) != b"ok":
        inconclusive("the bank refused the mute")
    time.sleep(1.2)
    # Phase C: heartbeats back (Lely resolves the timeout), then the process is killed.
    harness["t_unmute_c"] = time.monotonic()
    if client.ask(f"hb_mute {node} 0".encode()) != b"ok":
        inconclusive("the bank refused the unmute")
    peer.wait_for(lambda e: e.get("event") == "hb" and e.get("node") == node and
                  not e.get("occurred") and e["t"] > harness["t_unmute_c"], 2,
                  "Lely's heartbeat resolved for node 2")
    time.sleep(0.4)
    harness["bank_killed"] = bank.kill()
    time.sleep(2.2)
    harness["t_end"] = time.monotonic()
    quit_console(peer)
    judge(observe("M2", host, device, peer, bank, proxy, harness))


def test_sdo_expedited_and_aborts(capture, start_bank, start_proxy, start_peer):
    host, device, proxy, bank, peer, harness = start_rig(
        capture, start_bank, start_proxy, start_peer, "sdo_suite", node=NODE)
    peer.wait_for(lambda e: e.get("event") in ("scenario_done", "scenario_failed"),
                  PEER_DEADLINE_S + 5, "the end of sdo_suite")
    judge(observe("M3", host, device, peer, bank, proxy, harness))


def wire_us():
    """D5 time domain of valid_until: floor(CLOCK_MONOTONIC ns / 1000) mod 2^32."""
    return (time.monotonic_ns() // 1000) & 0xFFFFFFFF


def test_pdo_bytes_and_lengths_seen_by_peer(capture, start_bank, start_proxy, start_peer, peer):
    count = 50
    host, device, proxy, bank, lely, harness = start_rig(
        capture, start_bank, start_proxy, start_peer, "pdo_stream", node=NODE, count=count,
        before_peer=lambda bank: bank.trace.wait_for(
            lambda r: r.get("event") == "dispatch" and r.get("node") == NODE, 2,
            "a dispatch snapshot before the stream"))
    harness.update(count=count, injected=[])
    raw = peer(HOST_IF)
    rpdo1, rpdo2 = judges.pdo_layout("RPDO1"), judges.pdo_layout("RPDO2")

    def pair(seq, tag, rpdo1_dlc):
        f1 = bytes(judges.wire.pdo_frame(rpdo1, NODE, {"target_position": 0, "command_seq": seq,
                                                       "exec_tag": tag}, "t")["data"])[:rpdo1_dlc]
        f2 = bytes(judges.wire.pdo_frame(rpdo2, NODE, {"valid_until_us": wire_us() + 40_000,
                                                       "command_seq": seq, "exec_tag": tag},
                                         "t")["data"])
        t = raw.send(0x200 + NODE, f1)
        raw.send(0x300 + NODE, f2)
        harness["injected"] += [f1.hex(), f2.hex()]
        return t

    first = lely.wait_for(lambda e: e.get("event") == "stream" and e.get("seq") == 15, 5,
                          "stream update 15")
    # Seqs the stream never reaches: a well-formed pair of these is the newest at the next SYNC.
    harness["bad"] = {"seq": 1000, "t": pair(1000, first["tag"], 7)}
    lely.wait_for(lambda e: e.get("event") == "stream" and e.get("seq") == 30, 5,
                  "stream update 30")
    harness["control"] = {"seq": 2000, "t": pair(2000, first["tag"], 8)}
    lely.wait_for(lambda e: e.get("event") in ("scenario_done", "scenario_failed"),
                  PEER_DEADLINE_S + 5, "the end of pdo_stream")
    judge(observe("M4", host, device, lely, bank, proxy, harness))


def test_sync_application_and_sampling_order(capture, start_bank, start_proxy, start_peer):
    count = 50
    host, device, proxy, bank, peer, harness = start_rig(
        capture, start_bank, start_proxy, start_peer, "pdo_stream", node=NODE, count=count)
    harness["count"] = count
    peer.wait_for(lambda e: e.get("event") in ("scenario_done", "scenario_failed"),
                  PEER_DEADLINE_S + 5, "the end of pdo_stream")
    judge(observe("M5", host, device, peer, bank, proxy, harness))


def test_emcy_production_and_reception(capture, start_bank, start_proxy, start_peer,
                                       inject_client):
    host, device, proxy, bank, peer, harness = start_rig(
        capture, start_bank, start_proxy, start_peer, "console", node=NODE)
    wait_booted(peer)
    client = inject_client(bank)
    acks = []
    t = peer.command(f"nmt {NODE} 1")["t"]  # Operational: RPDO3 carries the Fault Reset.
    wait_state(peer, 5, t)
    harness["t_f1"] = time.monotonic()
    acks.append(client.ask(f"fault {NODE} 8611".encode()).decode())
    peer.wait_for(lambda e: e.get("event") == "emcy" and e["t"] > harness["t_f1"], 1,
                  "Lely's emcy for the fault")
    peer.command(f"read {NODE} 0x1001 0 u8")
    harness["t_f2"] = time.monotonic()
    acks.append(client.ask(f"fault {NODE} 1234".encode()).decode())
    time.sleep(0.3)
    harness["t_reset"] = peer.command(f"cw {NODE} 0x80")["t"]
    time.sleep(0.3)
    harness["t_stop"] = peer.command(f"nmt {NODE} 2")["t"]
    wait_state(peer, 4, harness["t_stop"])
    harness["t_f3"] = time.monotonic()
    acks.append(client.ask(f"fault {NODE} 8611".encode()).decode())
    time.sleep(0.3)
    harness["t_end"] = time.monotonic()
    harness["acks"] = acks
    quit_console(peer)
    judge(observe("M6", host, device, peer, bank, proxy, harness))
