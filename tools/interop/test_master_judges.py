# SPDX-License-Identifier: Apache-2.0
"""The master interop judges (N1-N7) on recorded observations, without vcan.

tools/interop/recorded/master/n1..n7.json.gz are observations of passing runs of
test_master_matrix.py (written with INTEROP_RECORD_DIR); n5_type255.json.gz is N5's sequence with
RPDO1 type 255 (applied on reception), recorded by the control test beside it. Each passing
recording passes its judge unchanged (the control). Each edit below breaks one thing a row
asserts, and the judge must say so: a broken pass condition is FAIL, a missing precondition
INCONCLUSIVE, never PASS.
"""
import copy
import gzip
import json
import pathlib

import pytest

import canopennode_facts as F
import cia301
import judges

RECORDED = pathlib.Path(__file__).parent / "recorded" / "master"
ROWS = ("N1", "N2", "N3", "N4", "N5", "N6", "N7")
NODE = F.NODE_ID
HB = cia301.HEARTBEAT_BASE + NODE
TPDO1 = cia301.TPDO_BASE[1] + NODE
SDO_TX = cia301.SDO_TX_BASE + NODE
_CACHE = {}


def load(name):
    if name not in _CACHE:
        with gzip.open(RECORDED / f"{name.lower()}.json.gz", "rt", encoding="utf-8") as f:
            _CACHE[name] = json.load(f)
    return copy.deepcopy(_CACHE[name])


def expect(obs, status, text):
    got, why = judges.verdict(obs)
    assert got == status and text in why, f"{got}: {why}"


def cli(obs, kind, **fields):
    return [e for e in obs["cli"] if e.get("event") == kind and
            all(e.get(k) == v for k, v in fields.items())]


def cmd_t(obs, n, kind="cmd"):
    return cli(obs, kind, n=n)[0]["t"]


def sdo_event(obs, n):
    return judges.cn_sdo(obs, n)


def nmt_device_time(obs, k):
    return [t for t, c, _ in obs["device"] if c == cia301.NMT_COB][k]


def add_frame(obs, side, t, cob, data):
    obs[side].append([t, cob, data.hex()])
    obs[side].sort(key=lambda f: f[0])


def edit_frames(obs, side, cob, t0, t1, new_hex=None, drop=False, first_only=True):
    """Replaces (or drops) the frames of `cob` in (t0, t1) on one side; returns how many."""
    out, hit = [], 0
    for f in obs[side]:
        if f[1] == cob and t0 < f[0] < t1 and (not first_only or hit == 0):
            hit += 1
            if drop:
                continue
            f = [f[0], f[1], new_hex]
        out.append(f)
    obs[side] = out
    assert hit, f"no frame {cob:#x} in ({t0}, {t1}) on {side}"
    return hit


@pytest.mark.parametrize("row", ROWS)
def test_recorded_observation_passes(row):
    status, text = judges.verdict(load(row))
    assert status == "PASS", f"{row} {status}: {text}"


@pytest.mark.parametrize("row", ROWS)
def test_proxy_with_rules_is_inconclusive(row):
    obs = load(row)
    obs["proxy_exit"]["rules"] = [{"matched": 0}]
    expect(obs, "INCONCLUSIVE", "ran with rules")


PRECONDITIONS = {
    "no start line": (lambda o: o.__setitem__("cli", [e for e in o["cli"]
                                                      if e["event"] != "start"]),
                      "no single start line"),
    "events lost": (lambda o: o["cli"][-1].__setitem__("event_overflows", 1), "lost events"),
    "no scenario end": (lambda o: o.__setitem__("cli", [
        e for e in o["cli"] if e["event"] not in ("scenario_done", "scenario_failed")]),
        "no single scenario_"),
    "proxy trace incomplete": (lambda o: o["proxy_exit"].__setitem__("trace_overflow", 1),
                               "trace incomplete"),
    "proxy send failed": (lambda o: o["proxy_exit"]["send_failed"].__setitem__("h2d", 1),
                          "lost frames"),
    "empty capture": (lambda o: o.__setitem__("device", []), "an empty capture"),
    "no canopend": (lambda o: o.__setitem__("canopend", []), "no canopend record"),
    "canopend never running": (lambda o: o["canopend"][0].__setitem__("log", []),
                               "never reported running"),
}


@pytest.mark.parametrize("row", ROWS)
@pytest.mark.parametrize("what", list(PRECONDITIONS))
def test_missing_precondition_is_inconclusive(row, what):
    obs = load(row)
    edit, text = PRECONDITIONS[what]
    edit(obs)
    expect(obs, "INCONCLUSIVE", text)


@pytest.mark.parametrize("row", ROWS)
def test_cli_without_exit_record_is_inconclusive(row):
    obs = load(row)
    obs["cli"] = [e for e in obs["cli"] if e["event"] != "exit"]
    expect(obs, "INCONCLUSIVE", "last line is not exit")


# ---- N1 ----

def test_n1_wrong_heartbeat_after_stop_fails():
    obs = load("N1")
    t = nmt_device_time(obs, 1)
    edit_frames(obs, "device", HB, t + 0.01, t + 1.0, "05")
    expect(obs, "FAIL", "after NMT 0x02 canopend's heartbeat")


def test_n1_master_state_disagreeing_with_the_wire_fails():
    obs = load("N1")
    n = obs["harness"]["nmt"][0][1]
    i = next(i for i, e in enumerate(obs["cli"]) if e["event"] == "cmd" and e["n"] == n)
    state = next(e for e in obs["cli"][i:] if e["event"] == "state")
    state["state"] = cia301.STATE_PRE_OPERATIONAL
    expect(obs, "FAIL", "after NMT 0x01 our master reports")


def test_n1_missing_canopend_log_line_fails():
    obs = load("N1")
    log = obs["canopend"][0]["log"]
    obs["canopend"][0]["log"] = [x for x in log if x[1] != F.LOG_NMT["stopped"]]
    expect(obs, "FAIL", "after NMT 0x02 canopend logged")


def test_n1_tpdo_in_pre_operational_fails():
    obs = load("N1")
    t = nmt_device_time(obs, 0) - 0.003
    add_frame(obs, "device", t, TPDO1, bytes(8))
    expect(obs, "FAIL", "TPDO1 in state 0x7f")


def test_n1_missing_tpdo_in_operational_fails():
    obs = load("N1")
    t0, t1 = nmt_device_time(obs, 0), nmt_device_time(obs, 1)
    edit_frames(obs, "device", TPDO1, t0 + 0.05, t1, drop=True)
    expect(obs, "FAIL", "SYNC periods drew TPDO1")


def test_n1_tpdo_not_decoded_by_the_master_fails():
    obs = load("N1")
    pdo = cli(obs, "pdo")
    obs["cli"].remove(pdo[2])
    expect(obs, "FAIL", "did not decode once")


def test_n1_sdo_answer_in_stopped_fails():
    obs = load("N1")
    n = [n for state, n in obs["harness"]["sdo"] if state == cia301.STATE_STOPPED][0]
    t = cmd_t(obs, n) + 0.01
    answer = cia301.sdo_upload_response(NODE, 0x1000, 0, 4, 0)[1]
    add_frame(obs, "device", t, SDO_TX, answer)
    expect(obs, "FAIL", "an SDO answer in Stopped")


def test_n1_sdo_ok_in_stopped_fails():
    obs = load("N1")
    n = [n for state, n in obs["harness"]["sdo"] if state == cia301.STATE_STOPPED][0]
    e = sdo_event(obs, n)
    e["event"], e["value"], e["size"] = "sdo_ok", 0, 4
    expect(obs, "FAIL", "in Stopped our master reports")


def test_n1_reset_node_exit_code_fails():
    obs = load("N1")
    obs["canopend"][0]["exit_code"] = 1
    obs["harness"]["reset_node"]["exit_code"] = 1
    expect(obs, "FAIL", "at Reset Node, pinned 0")


def test_n1_no_boot_up_after_the_restart_fails():
    obs = load("N1")
    t = obs["canopend"][1]["started"]
    edit_frames(obs, "device", HB, t, t + 1.0, drop=True)
    expect(obs, "FAIL", "after the restart [")


def test_n1_unrequested_nmt_from_the_master_fails():
    obs = load("N1")
    t = nmt_device_time(obs, 0) + 0.05
    add_frame(obs, "host", t, cia301.NMT_COB, bytes([cia301.NMT_RESET_NODE, NODE]))
    expect(obs, "FAIL", "NMT frames from our master")


# ---- N2 ----

def test_n2_boot_timeout_late_fails():
    obs = load("N2")
    cli(obs, "boot_timeout")[0]["at_ns"] += 200_000_000
    expect(obs, "FAIL", "boot_timeout")


def test_n2_nmt_from_the_master_fails():
    obs = load("N2")
    t = obs["harness"]["sigstop"] + 0.45
    add_frame(obs, "host", t, cia301.NMT_COB, bytes([cia301.NMT_RESET_COMMUNICATION, NODE]))
    expect(obs, "FAIL", "our master sent NMT")


def test_n2_heartbeat_lost_too_late_fails():
    obs = load("N2")
    cli(obs, "hb_lost")[0]["at_ns"] += 160_000_000
    expect(obs, "FAIL", "hb_lost")


def test_n2_no_heartbeat_lost_fails():
    obs = load("N2")
    obs["cli"] = [e for e in obs["cli"] if e["event"] != "hb_lost"]
    expect(obs, "FAIL", "hb_lost events")


def test_n2_no_heartbeat_resumed_fails():
    obs = load("N2")
    t = obs["harness"]["sigcont"]
    obs["cli"] = [e for e in obs["cli"] if not (e["event"] == "hb_resumed" and e["t"] > t)]
    expect(obs, "FAIL", "hb_resumed")


def test_n2_wrong_heartbeat_time_fails():
    obs = load("N2")
    sdo_event(obs, obs["harness"]["read_1017"])["value"] = 0
    expect(obs, "FAIL", "0x1017 reads")


def test_n2_slave_not_silenced_is_inconclusive():
    obs = load("N2")
    t = (obs["harness"]["sigstop"] + obs["harness"]["sigcont"]) / 2
    add_frame(obs, "device", t, HB, bytes([cia301.STATE_PRE_OPERATIONAL]))
    expect(obs, "INCONCLUSIVE", "SIGSTOP did not take")


# ---- N3 ----

def test_n3_wrong_device_type_fails():
    obs = load("N3")
    sdo_event(obs, obs["harness"]["device_type"])["value"] = 0x20192
    expect(obs, "FAIL", "0x1000:0 reads")


def test_n3_wire_response_not_matching_fails():
    obs = load("N3")
    n = obs["harness"]["vendor"]
    edit_frames(obs, "host", SDO_TX, cmd_t(obs, n), cmd_t(obs, n, "cmd_done"),
                cia301.sdo_upload_response(NODE, 0x1018, 1, 4, 7)[1].hex())
    expect(obs, "FAIL", "0x1018:1 reads")


@pytest.mark.parametrize("label", list(judges.CN_ABORTS))
def test_n3_abort_code_decoded_wrong_fails(label):
    obs = load("N3")
    sdo_event(obs, obs["harness"]["aborts"][label])["code"] = cia301.ABORT_LENGTH
    expect(obs, "FAIL", f"{label}: our master reports")


def test_n3_abort_on_the_wire_differs_fails():
    obs = load("N3")
    n = obs["harness"]["aborts"]["read_only_download"]
    edit_frames(obs, "host", SDO_TX, cmd_t(obs, n), cmd_t(obs, n, "cmd_done"),
                cia301.sdo_abort(NODE, 0x1000, 0, cia301.ABORT_LENGTH)[1].hex())
    expect(obs, "FAIL", "read_only_download: the wire answer")


def test_n3_heartbeat_lost_fails():
    obs = load("N3")
    lost = dict(cli(obs, "boot")[0])
    lost["event"] = "hb_lost"
    obs["cli"].insert(-1, lost)
    expect(obs, "FAIL", "heartbeat consumer events")


def test_n3_heartbeat_period_unchanged_fails():
    obs = load("N3")
    t0 = cmd_t(obs, obs["harness"]["write_400"])
    t1 = cmd_t(obs, obs["harness"]["write_100"])
    for k in range(1, int((t1 - t0) / 0.1)):
        add_frame(obs, "device", t0 + 0.1 * k - 0.05, HB, bytes([cia301.STATE_PRE_OPERATIONAL]))
    expect(obs, "FAIL", "heartbeat at 0x1017 = 400")


# ---- N4 ----

def test_n4_master_pdo_decoding_disabled_fails():
    obs = load("N4")
    obs["cli"] = [e for e in obs["cli"] if e["event"] != "pdo"]
    expect(obs, "FAIL", "our master decoded 0 pdo events")


def test_n4_swapped_field_fails():
    obs = load("N4")
    for e in cli(obs, "pdo"):
        e["fields"] = e["fields"][::-1]
    expect(obs, "FAIL", "our master decoded")


def test_n4_tpdo_bytes_not_the_sdo_values_fails():
    obs = load("N4")
    t = [t for t, c, _ in obs["host"] if c == TPDO1][0]
    edit_frames(obs, "host", TPDO1, t - 0.001, t + 0.001, "2401008056040080")
    expect(obs, "FAIL", "the SDO values encode as")


def test_n4_rpdo_dlc_on_the_device_fails():
    obs = load("N4")
    rpdo = cia301.RPDO_BASE[1] + NODE
    t = [t for t, c, _ in obs["device"] if c == rpdo][0]
    edit_frames(obs, "device", rpdo, t - 0.001, t + 0.001, "230100")
    expect(obs, "FAIL", "RPDO1 DLCs on vcan_device")


def test_n4_wrong_length_emcy_fails():
    obs = load("N4")
    t = [t for t, c, _ in obs["device"] if c == F.EMCY_COB_ID_1014][0]
    edit_frames(obs, "device", F.EMCY_COB_ID_1014, t - 0.001, t + 0.001, "2082000402000000")
    expect(obs, "FAIL", "after the DLC 1 RPDO1 canopend sent")


def test_n4_rpdo_not_applied_fails():
    obs = load("N4")
    sdo_event(obs, obs["harness"]["read_rpdo"])["value"] = 0x80000123  # The value before.
    expect(obs, "FAIL", "0x1280:01 reads")


# ---- N5 ----

def test_n5_type_255_recording_fails():
    """The committed negative fixture: RPDO1 applied on reception."""
    obs = load("n5_type255")
    assert obs["harness"]["rpdo_type"] == 255
    expect(obs, "FAIL", "visible by SDO before the first SYNC after its RPDO")


def test_n5_one_more_sync_before_the_rpdos_flips_the_prediction_fails():
    """A SYNC canopend saw that the judge did not count would flip every prediction: the
    applying SYNC is predicted from the pinned parity, not accepted wherever it falls."""
    obs = load("N5")
    boot = [t for t, c, d in obs["device"] if c == HB and d == "00"][0]
    add_frame(obs, "device", boot + 0.0005, cia301.SYNC_COB, b"")
    expect(obs, "FAIL", "the pinned parity predicts")


def test_n5_value_visible_before_its_applying_sync_fails():
    obs = load("N5")
    value = obs["harness"]["values"][2]
    a = value["attempts"][-1]
    sdo_event(obs, a["before"])["value"] = F.LOOPBACK_DISABLED | value["k"]
    expect(obs, "FAIL", "visible by SDO before")


def test_n5_applied_at_the_wrong_sync_fails():
    obs = load("N5")
    value = next(v for v in obs["harness"]["values"] if len(v["attempts"]) == 2)
    sdo_event(obs, value["attempts"][0]["after"])["value"] = F.LOOPBACK_DISABLED | value["k"]
    expect(obs, "FAIL", "the pinned parity predicts")


def test_n5_tpdo_without_the_applied_value_fails():
    obs = load("N5")
    value = obs["harness"]["values"][0]
    step = value["attempts"][-1]["step"]
    t = cmd_t(obs, step)
    edit_frames(obs, "host", TPDO1, t, t + 0.05, F.LOOPBACK_DISABLED.to_bytes(4, "little").hex())
    expect(obs, "FAIL", "TPDO1 after SYNC")


def test_n5_master_decoding_the_old_value_fails():
    obs = load("N5")
    value = obs["harness"]["values"][3]
    t = cmd_t(obs, value["attempts"][-1]["step"])
    pdo = [e for e in cli(obs, "pdo") if e["t"] > t][0]
    pdo["fields"][0]["raw"] = F.LOOPBACK_DISABLED
    expect(obs, "FAIL", "our master decoded")


def test_n5_fewer_than_twenty_values_fails():
    obs = load("N5")
    obs["harness"]["values"] = obs["harness"]["values"][:19]
    expect(obs, "FAIL", "at least 20 needed")


def test_n5_sync_during_canopend_start_is_inconclusive():
    obs = load("N5")
    boot = [t for t, c, d in obs["device"] if c == HB and d == "00"][0]
    add_frame(obs, "device", boot - 0.0001, cia301.SYNC_COB, b"")
    expect(obs, "INCONCLUSIVE", "a SYNC while canopend initialised")


# ---- N6 ----

def test_n6_emcy_code_decoded_wrong_fails():
    obs = load("N6")
    cli(obs, "emcy")[0]["code"] = F.EMCY_PDO_LENGTH_LONG
    expect(obs, "FAIL", "our master decoded code")


def test_n6_emcy_register_decoded_wrong_fails():
    obs = load("N6")
    cli(obs, "emcy")[0]["register"] = cia301.ERROR_REGISTER_GENERIC
    expect(obs, "FAIL", "our master decoded code")


def test_n6_error_reset_not_decoded_fails():
    obs = load("N6")
    obs["cli"].remove(cli(obs, "emcy")[1])
    expect(obs, "FAIL", "2 EMCY on vcan_host, 1 decoded")


# ---- N7 ----

def activate_done(obs):
    n = [e for e in cli(obs, "cmd") if e["line"] == "activate"][0]["n"]
    return cli(obs, "cmd_done", n=n)[0]


def test_n7_activation_not_refused_fails():
    obs = load("N7")
    activate_done(obs)["ok"] = True
    expect(obs, "FAIL", "activation was not refused")


@pytest.mark.parametrize("capability", list(judges.CN_GENERIC_MISSING))
def test_n7_capability_missing_from_the_refusal_fails(capability):
    obs = load("N7")
    done = activate_done(obs)
    done["missing"] = [m for m in done["missing"] if m["capability"] != capability]
    expect(obs, "FAIL", "activation refused listing")


def test_n7_unknown_reported_as_unsupported_fails():
    obs = load("N7")
    for m in activate_done(obs)["missing"]:
        m["support"] = "UNSUPPORTED"
    expect(obs, "FAIL", "activation refused listing")


def test_n7_probe_not_aborted_fails():
    obs = load("N7")
    cli(obs, "sdo_abort", index=0x2012)[0]["code"] = cia301.ABORT_LENGTH
    expect(obs, "FAIL", "0x2012 probes")


def test_n7_nmt_start_after_the_refusal_fails():
    obs = load("N7")
    t = activate_done(obs)["t"] + 0.05
    add_frame(obs, "host", t, cia301.NMT_COB, bytes([cia301.NMT_START, 0]))
    expect(obs, "FAIL", "NMT Start on vcan_host")


def test_n7_rpdo_after_the_refusal_fails():
    obs = load("N7")
    t = activate_done(obs)["t"] + 0.05
    add_frame(obs, "device", t, cia301.RPDO_BASE[2] + 3, bytes(8))
    expect(obs, "FAIL", "RPDO on vcan_device")


# ---- each remaining pass condition of a row, broken once ----

def cmd_index(obs, n):
    return next(i for i, e in enumerate(obs["cli"]) if e["event"] == "cmd" and e["n"] == n)


def event_after(obs, n, kind):
    i = cmd_index(obs, n)
    return next(e for e in obs["cli"][i:] if e["event"] == kind and e.get("node") == NODE)


def n1_nmt(obs, cs):
    return next(n for c, n in obs["harness"]["nmt"] if c == cs)


def test_n1_nmt_not_forwarded_to_the_device_fails():
    obs = load("N1")
    t = nmt_device_time(obs, 1)
    edit_frames(obs, "device", cia301.NMT_COB, t - 0.001, t + 0.001, drop=True)
    expect(obs, "FAIL", "NMT frames on vcan_device")


def test_n1_operational_tpdo_of_other_length_fails():
    obs = load("N1")
    t0, t1 = nmt_device_time(obs, 0), nmt_device_time(obs, 1)
    edit_frames(obs, "device", TPDO1, t0 + 0.05, t1, "00000080")
    expect(obs, "FAIL", "DLC other than 8")


def test_n1_heartbeat_before_start_not_pre_operational_fails():
    obs = load("N1")
    boot = [t for t, c, d in obs["device"] if c == HB and d == "00"][0]
    edit_frames(obs, "device", HB, boot + 0.001, nmt_device_time(obs, 0), "05")
    expect(obs, "FAIL", "heartbeat before Start")


def test_n1_no_boot_up_after_reset_communication_fails():
    obs = load("N1")
    t = nmt_device_time(obs, 3)
    edit_frames(obs, "device", HB, t, t + 0.1, drop=True)
    expect(obs, "FAIL", "after Reset Communication [")


def test_n1_process_ended_at_reset_communication_fails():
    obs = load("N1")
    obs["canopend"][0]["exited"] = nmt_device_time(obs, 3) + 0.01
    expect(obs, "FAIL", "canopend ended at Reset Communication")


def test_n1_master_missing_boot_after_reset_communication_fails():
    obs = load("N1")
    obs["cli"].remove(event_after(obs, n1_nmt(obs, cia301.NMT_RESET_COMMUNICATION), "boot"))
    expect(obs, "FAIL", "no boot-up after Reset Communication")


def test_n1_master_state_after_reset_communication_fails():
    obs = load("N1")
    event_after(obs, n1_nmt(obs, cia301.NMT_RESET_COMMUNICATION), "state")["state"] = 5
    expect(obs, "FAIL", "after Reset Communication our master reports")


def test_n1_reset_communication_not_logged_fails():
    obs = load("N1")
    t = nmt_device_time(obs, 3)
    log = obs["canopend"][0]["log"]
    obs["canopend"][0]["log"] = [x for x in log if not (x[0] > t and
                                                        x[1] == F.LOG_NMT["initializing"])]
    expect(obs, "FAIL", "at Reset Communication")


def test_n1_reset_node_without_finished_log_fails():
    obs = load("N1")
    obs["canopend"][0]["log"] = [x for x in obs["canopend"][0]["log"] if x[1] != F.LOG_FINISHED]
    expect(obs, "FAIL", "last log line")


def test_n1_frames_between_reset_node_and_restart_fail():
    obs = load("N1")
    t = (nmt_device_time(obs, 4) + obs["canopend"][1]["started"]) / 2
    add_frame(obs, "device", t, HB, bytes([cia301.STATE_PRE_OPERATIONAL]))
    expect(obs, "FAIL", "between Reset Node and the restart")


def test_n1_restart_not_logged_fails():
    obs = load("N1")
    obs["canopend"][1]["log"] = [x for x in obs["canopend"][1]["log"]
                                 if x[1] != F.LOG_NMT["initializing"]]
    expect(obs, "FAIL", "the restarted canopend logged")


def test_n1_master_missing_boot_after_the_restart_fails():
    obs = load("N1")
    obs["cli"].remove(event_after(obs, n1_nmt(obs, cia301.NMT_RESET_NODE), "boot"))
    expect(obs, "FAIL", "no boot-up after the restart")


def test_n1_master_state_after_the_restart_fails():
    obs = load("N1")
    event_after(obs, n1_nmt(obs, cia301.NMT_RESET_NODE), "state")["state"] = 5
    expect(obs, "FAIL", "after the restart our master reports")


def test_n1_sdo_unanswered_in_pre_operational_fails():
    obs = load("N1")
    n = obs["harness"]["sdo"][0][1]
    e = sdo_event(obs, n)
    e["event"], e["code"] = "sdo_timeout", cia301.ABORT_TIMEOUT
    expect(obs, "FAIL", "in 0x7f the SDO got sdo_timeout")


def test_n1_no_client_abort_after_the_stopped_timeout_fails():
    obs = load("N1")
    n = [n for state, n in obs["harness"]["sdo"] if state == cia301.STATE_STOPPED][0]
    rx = cia301.SDO_RX_BASE + NODE
    t0, t1 = cmd_t(obs, n), cmd_t(obs, n, "cmd_done")
    obs["device"] = [f for f in obs["device"]
                     if not (f[1] == rx and t0 < f[0] < t1 and f[2].startswith("80"))]
    expect(obs, "FAIL", "our master's abort after the timeout")


def test_n1_pdo_decoded_with_no_frame_in_stopped_fails():
    obs = load("N1")
    t = nmt_device_time(obs, 1) + 0.1
    pdo = dict(cli(obs, "pdo")[0])
    pdo["sync_index"] = judges.cn_host_sync_index(obs, t)
    pdo["at_ns"], pdo["t"] = int(t * 1e9), t
    obs["cli"].insert(-1, pdo)
    expect(obs, "FAIL", "pdo events with no TPDO1 frame in state 0x04")


def test_n2_two_boot_timeouts_fail():
    obs = load("N2")
    obs["cli"].insert(-1, dict(cli(obs, "boot_timeout")[0]))
    expect(obs, "FAIL", "2 boot_timeout events")


def test_n2_boot_up_after_sigcont_fails():
    obs = load("N2")
    t = obs["harness"]["sigcont"]
    edit_frames(obs, "host", HB, t, t + 1.0, "00")
    expect(obs, "FAIL", "after SIGCONT")


def test_n2_frame_before_canopend_started_is_inconclusive():
    obs = load("N2")
    add_frame(obs, "host", obs["canopend"][0]["started"] - 0.1, HB, b"\x7f")
    expect(obs, "INCONCLUSIVE", "before canopend started")


def test_n3_upload_of_another_object_fails():
    obs = load("N3")
    cli(obs, "cmd", n=obs["harness"]["device_type"])[0]["line"] = f"read {NODE} 0x1001 0 u32"
    expect(obs, "FAIL", "is 'read 2 0x1001 0 u32'")


def test_n3_upload_response_not_expedited_fails():
    obs = load("N3")
    n = obs["harness"]["device_type"]
    edit_frames(obs, "host", SDO_TX, cmd_t(obs, n), cmd_t(obs, n, "cmd_done"),
                "4100100004000000")
    expect(obs, "FAIL", "is no expedited upload response")


def test_n3_consumer_not_disabled_fails():
    obs = load("N3")
    cli(obs, "cmd", n=obs["harness"]["consumer_off"])[0]["line"] = f"hb {NODE} 250"
    expect(obs, "FAIL", "the consumer was not disabled")


def test_n3_heartbeat_time_written_with_the_consumer_on_fails():
    obs = load("N3")
    obs["harness"]["consumer_on"] = obs["harness"]["consumer_off"]
    expect(obs, "FAIL", "outside the disabled window")


def test_n3_other_heartbeat_time_written_fails():
    obs = load("N3")
    n = obs["harness"]["write_400"]
    cli(obs, "cmd", n=n)[0]["line"] = f"write {NODE} 0x1017 0 u16 0x191"
    expect(obs, "FAIL", "0x191")


def test_n3_heartbeat_time_download_aborted_fails():
    obs = load("N3")
    e = sdo_event(obs, obs["harness"]["write_400"])
    e["event"], e["code"] = "sdo_abort", F.ABORT_LENGTH_TOO_HIGH
    expect(obs, "FAIL", "download 0x1017 = 400")


def test_n3_no_heartbeat_after_re_enabling_fails():
    obs = load("N3")
    t = cmd_t(obs, obs["harness"]["consumer_on"])
    obs["host"] = [f for f in obs["host"] if not (f[1] == HB and f[0] > t)]
    expect(obs, "FAIL", "heartbeats after re-enabling")


def test_n3_abort_case_on_another_object_fails():
    obs = load("N3")
    n = obs["harness"]["aborts"]["unknown_subindex"]
    cli(obs, "cmd", n=n)[0]["line"] = f"read {NODE} 0x1018 6 u32"
    expect(obs, "FAIL", "unknown_subindex: line")


def test_n4_pdo_decoded_with_no_frame_fails():
    obs = load("N4")
    tpdo = [t for t, c, _ in obs["host"] if c == TPDO1]
    t = tpdo[4] + 0.0002
    edit_frames(obs, "host", TPDO1, t - 0.0004, t, drop=True)
    expect(obs, "FAIL", "pdo events with no TPDO1 frame")


def test_n4_master_rpdo_differs_from_the_device_fails():
    obs = load("N4")
    rpdo = cia301.RPDO_BASE[1] + NODE
    t = [t for t, c, _ in obs["host"] if c == rpdo][0]
    edit_frames(obs, "host", rpdo, t - 0.001, t + 0.001, "2401")
    expect(obs, "FAIL", "RPDO1 from our master")


def test_n4_state_change_after_the_length_error_fails():
    obs = load("N4")
    t = [t for t, c, d in obs["device"] if c == cia301.RPDO_BASE[1] + NODE and len(d) == 4][1]
    edit_frames(obs, "device", HB, t, t + 1.0, "7f")
    expect(obs, "FAIL", "the NMT state changed")


def test_n6_emcy_decoded_late_fails():
    obs = load("N6")
    cli(obs, "emcy")[0]["at_ns"] += 30_000_000
    expect(obs, "FAIL", "after the frame")


def test_n5_tpdo_after_a_pad_sync_fails():
    obs = load("N5")
    value = next(v for v in obs["harness"]["values"] if v["pads"])
    t = cmd_t(obs, value["pads"][0])
    edit_frames(obs, "host", TPDO1, t, t + 0.05, (F.LOOPBACK_DISABLED | 0x7FF).to_bytes(
        4, "little").hex())
    expect(obs, "FAIL", "no RPDO pending")


def test_n5_value_changed_before_the_sync_fails():
    obs = load("N5")
    value = obs["harness"]["values"][4]
    sdo_event(obs, value["attempts"][-1]["before"])["value"] = F.LOOPBACK_DISABLED | 0x7FF
    expect(obs, "FAIL", "before the SYNC, the old value")


def test_n5_harness_giving_up_early_fails():
    obs = load("N5")
    value = next(v for v in obs["harness"]["values"] if len(v["attempts"]) == 2)
    value["attempts"] = value["attempts"][:1]
    expect(obs, "FAIL", "was never applied")


def test_n7_probe_abort_missing_on_the_wire_fails():
    obs = load("N7")
    abort = cia301.sdo_abort(4, 0x2012, 0, F.ABORT_NO_OBJECT)[1].hex()
    obs["host"] = [f for f in obs["host"] if not (f[1] == cia301.SDO_TX_BASE + 4 and f[2] == abort)]
    expect(obs, "FAIL", "node 4: no 0x2012 abort on the wire")



def test_n1_operational_pdo_decoded_with_no_frame_on_the_host_fails():
    obs = load("N1")
    t0 = nmt_device_time(obs, 0)
    t = [t for t, c, _ in obs["host"] if c == TPDO1 and t > t0 + 0.05][0]
    edit_frames(obs, "host", TPDO1, t - 0.0001, t + 0.0001, drop=True)
    expect(obs, "FAIL", "pdo events with no TPDO1 frame: [")


def _drop_sync_event(o):
    o["cli"].remove(cli(o, "sync")[5])


def _n5_rpdo_after_the_read(o):
    a = o["harness"]["values"][0]["attempts"][0]
    t = cmd_t(o, a["step"]) - 0.0001
    rpdo = cia301.RPDO_BASE[1] + NODE
    first = [f for f in o["device"] if f[1] == rpdo][0]
    first[0] = t
    o["device"].sort(key=lambda f: f[0])


def _n1_no_restart(o):
    o["canopend"] = o["canopend"][:1]


def _n1_config_failed(o):
    e = sdo_event(o, o["harness"]["config"][0])
    e["event"], e["code"] = "sdo_abort", F.ABORT_NOT_MAPPABLE


def _n2_short_stop(o):
    o["harness"]["sigcont"] = o["harness"]["sigstop"] + 0.45


def _n3_no_400_interval(o):
    t0 = cmd_t(o, o["harness"]["write_400"])
    t1 = cmd_t(o, o["harness"]["write_100"])
    o["device"] = [f for f in o["device"] if not (f[1] == HB and t0 < f[0] < t1)]


def _n4_few_tpdos(o):
    t0 = nmt_device_time(o, 0)
    rpdo = [t for t, c, _ in o["device"] if c == cia301.RPDO_BASE[1] + NODE][0]
    keep = [t for t, c, _ in o["host"] if c == TPDO1 and t0 < t < rpdo][:3]
    drop = {t for t, c, _ in o["host"] if c == TPDO1 and t0 < t < rpdo} - set(keep)
    o["host"] = [f for f in o["host"] if not (f[1] == TPDO1 and f[0] in drop)]


def _n6_no_reset_emcy(o):
    t = [t for t, c, _ in o["host"] if c == F.EMCY_COB_ID_1014][1]
    o["host"] = [f for f in o["host"] if f[0] != t]


def _n7_five_nodes(o):
    o["canopend"] = o["canopend"][:5]


def _n7_cli_exit_0(o):
    o["harness"]["cli_rc"] = 0


def _n7_configure_failed(o):
    n = [e for e in cli(o, "cmd") if e["line"] == "configure"][0]["n"]
    cli(o, "cmd_done", n=n)[0]["ok"] = False


def _n7_short_tail(o):
    t = activate_done(o)["t"] + 0.1
    o["host"] = [f for f in o["host"] if f[0] < t]


def _line_not_carried_out(o):
    cli(o, "cmd_done", n=o["harness"]["config"][0])[0]["ok"] = False


PRECONDITION_EDITS = {
    "N1 a sync event lost": ("N1", _drop_sync_event, "sync_index does not count"),
    "N1 no restart": ("N1", _n1_no_restart, "Reset Node needs a restart"),
    "N1 configuration aborted": ("N1", _n1_config_failed, "TPDO1 configuration line"),
    "N1 console line failed": ("N1", _line_not_carried_out, "was not carried out"),
    "N2 stopped too briefly": ("N2", _n2_short_stop, "stopped only"),
    "N3 no heartbeat at 400": ("N3", _n3_no_400_interval, "no heartbeat interval"),
    "N4 too few TPDOs": ("N4", _n4_few_tpdos, "TPDO1 frames to judge"),
    "N5 RPDO after the read": ("N5", _n5_rpdo_after_the_read, "RPDO1 before the read"),
    "N5 a sync event lost": ("N5", _drop_sync_event, "sync_index does not count"),
    "N6 no error reset": ("N6", _n6_no_reset_emcy, "EMCY codes on vcan_host"),
    "N7 five nodes": ("N7", _n7_five_nodes, "canopend nodes"),
    "N7 CLI exit 0": ("N7", _n7_cli_exit_0, "the CLI exited 0"),
    "N7 configure failed": ("N7", _n7_configure_failed, "configure failed"),
    "N7 short tail": ("N7", _n7_short_tail, "observed after the refusal"),
}


@pytest.mark.parametrize("what", list(PRECONDITION_EDITS))
def test_row_precondition_missing_is_inconclusive(what):
    row, edit, text = PRECONDITION_EDITS[what]
    obs = load(row)
    edit(obs)
    expect(obs, "INCONCLUSIVE", text)
