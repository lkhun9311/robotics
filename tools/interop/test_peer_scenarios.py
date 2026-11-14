# SPDX-License-Identifier: Apache-2.0
"""The Lely peer's scenarios against the running vdrive_bank (E3b Task 5).

Lely (lely_peer on vcan_host) reaches the bank (vcan_device) through bus_fault_proxy without
rules. Each scenario must end by itself with exit 0 and a `scenario_done` line; a scenario that
could not perform a step prints `scenario_failed` and exits 1. These tests check that the
scenarios run and do what they claim on the bank side; the CiA 301 verdicts on what the drives
answered belong to the interop matrix (Task 6).
"""
import pytest

from conftest import DEVICE_IF, NODES, inconclusive

NODE = 3
OPERATIONAL, STOPPED, PRE_OPERATIONAL = 5, 4, 127  # Lely NmtState values in `state` events.


def run_scenario(start_peer, scenario, timeout=10, **kwargs):
    """Starts the peer, waits for it to end by itself; returns (peer, scenario_done event)."""
    peer = start_peer(scenario, **kwargs)
    peer.wait_for(lambda e: e.get("event") in ("scenario_done", "scenario_failed"), timeout,
                  f"end of scenario {scenario}")
    rc = peer.wait_exit(timeout=5)
    failed = peer.of("scenario_failed")
    assert not failed, f"{scenario} failed: {failed}; stderr: {peer.stderr[-500:]!r}"
    assert rc == 0, f"{scenario} exited {rc}; stderr: {peer.stderr[-500:]!r}"
    done = peer.of("scenario_done")
    assert len(done) == 1 and done[0]["scenario"] == scenario, done
    assert peer.of("exit"), "no clean exit line"
    return peer, done[0]


def drive_records(bank, kind, node):
    return [r for r in bank.trace.poll()
            if r.get("event") == "drive" and r.get("kind") == kind and r.get("node") == node]


@pytest.mark.parametrize("scenario", ["nmt_sequence", "sdo_suite", "commit_and_enable",
                                      "pdo_stream", "emcy_watch", "idle"])
def test_peer_scenarios_run(scenario, start_bank, proxy, start_peer, inject_client, capture):
    cap = capture(DEVICE_IF)  # What the drives saw, encoded by Lely.
    bank = start_bank(inject=True)
    bank.trace.wait_for(lambda r: r.get("event") == "start", 2, "bank start record")

    if scenario == "idle":
        peer, done = run_scenario(start_peer, "idle", duration_ms=500)
        assert {e["node"] for e in peer.of("boot")} == set(NODES), peer.of("boot")

    elif scenario == "nmt_sequence":
        peer, done = run_scenario(start_peer, "nmt_sequence", node=NODE)
        # Start, Stop, Enter Pre-operational, Start, Reset Communication, Reset Node.
        assert done["commands"] == 6, done
        states = [e["state"] for e in peer.of("state", node=NODE)]
        for st in (OPERATIONAL, STOPPED, PRE_OPERATIONAL):
            assert st in states, (st, states)
        assert len([e for e in peer.of("boot", node=NODE)]) >= 3, peer.of("boot", node=NODE)

    elif scenario == "sdo_suite":
        peer, done = run_scenario(start_peer, "sdo_suite", node=NODE)
        assert done["timeout"] == 0, done
        assert done["ok"] >= 7 and done["abort"] >= 5, done
        assert done["ok"] == len(peer.of("sdo_ok", node=NODE)), done
        assert done["abort"] == len(peer.of("sdo_abort", node=NODE)), done
        device_type = peer.of("sdo_ok", node=NODE, index=0x1000, sub=0)
        assert device_type and device_type[0]["value"] == 0x00020192, device_type

    elif scenario == "commit_and_enable":
        peer, done = run_scenario(start_peer, "commit_and_enable", node=NODE)
        assert done["tag"] != 0 and done["attempt_id"] >= 1, done
        assert done["statusword"] & 0x6F == 0x27, done  # Operation Enabled, seen by Lely.
        frames = cap.stop()
        # RPDO3: the controlwords as Lely encoded them from its remote mapping (DLC 2).
        assert [d for _, cob, d in frames if cob == 0x400 + NODE] == \
            [b"\x06\x00", b"\x07\x00", b"\x0f\x00"], frames

    elif scenario == "pdo_stream":
        peer, done = run_scenario(start_peer, "pdo_stream", node=NODE, count=50)
        assert done["sent"] == 50, done
        frames = cap.stop()
        tag = done["tag"].to_bytes(2, "little")
        # RPDO1 (target, seq, tag) and RPDO2 (valid_until, seq, tag) of this tag on the wire.
        seqs1 = {int.from_bytes(d[4:6], "little") for _, cob, d in frames
                 if cob == 0x200 + NODE and len(d) == 8 and d[6:8] == tag}
        seqs2 = {int.from_bytes(d[4:6], "little") for _, cob, d in frames
                 if cob == 0x300 + NODE and len(d) == 8 and d[6:8] == tag}
        if seqs1 != set(range(1, 51)) or seqs2 != set(range(1, 51)):
            inconclusive(f"command population on the wire: RPDO1 seqs {sorted(seqs1)}, "
                         f"RPDO2 seqs {sorted(seqs2)}")
        bank.stop()  # The complete trace (exit record, no overflow) before it is judged.
        actuated = drive_records(bank, "actuated", NODE)
        tags = {r["tag"] for r in actuated}
        print(f"pdo_stream: {len(actuated)} actuated of 50; Lely last applied_seq "
              f"{done['applied_seq']}")
        assert tags == {done["tag"]}, tags
        assert len(actuated) >= 45, f"{len(actuated)} kActuated of 50 sent"

    elif scenario == "emcy_watch":
        peer = start_peer("emcy_watch", duration_ms=1500)
        peer.wait_for(lambda e: e.get("event") == "boot" and e.get("node") == NODE, 5,
                      f"boot of node {NODE}")
        client = inject_client(bank)
        if client.ask(f"fault {NODE} 4321".encode()) != b"ok":
            inconclusive("the bank refused the injected fault")
        peer.wait_for(lambda e: e.get("event") == "scenario_done", 5, "scenario_done")
        assert peer.wait_exit(timeout=5) == 0, peer.stderr
        emcy = peer.of("emcy", node=NODE, code=0x4321)
        assert emcy, peer.of("emcy")
        done = peer.of("scenario_done")[0]
        assert done["emcy"] == len(peer.of("emcy")) >= 1, done

    exit_record = bank.stop()
    assert exit_record["signal"] == 15, exit_record


def test_peer_scenario_failure_is_reported(start_peer):
    """A step the peer cannot perform is scenario_failed with its reason and exit 1, never a
    scenario_done: here the node is not a slave in the master DCF."""
    peer = start_peer("sdo_suite", node=9)
    peer.wait_for(lambda e: e.get("event") in ("scenario_done", "scenario_failed"), 5,
                  "end of the scenario")
    assert peer.wait_exit(timeout=5) == 1, peer.stderr
    failed = peer.of("scenario_failed")
    assert len(failed) == 1 and "node 9" in failed[0]["reason"], failed
    assert not peer.of("scenario_done")
    assert not peer.of("sdo_ok") and not peer.of("sdo_abort")
