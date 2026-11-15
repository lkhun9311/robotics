# SPDX-License-Identifier: Apache-2.0
"""The E3 exit interop matrix, our master against an independent slave (E3c Task 8, rows N1-N7).

canopen_master_cli on vcan_host <-> bus_fault_proxy (no rules) <-> vcan_device <-> canopend
(CANopenNode 76b43c8, node 2; N7 runs six of them as nodes 1-6). The tests drive our master
through its console (stdin) or its activate_hal scenario, capture both links, keep canopend's log
(its own report of its NMT state) and judge the observation with judges.judge_n1..n7. The
expectations come from cia301.py and canopennode_facts.py (calibrated by
test_canopennode_config.py), never from the master's code. With INTEROP_RECORD_DIR set every
observation is written there first; tools/interop/recorded/master/ holds passing ones, judged
again without vcan by test_master_judges.py and tools/evidence/rejudge.py.

Every console line is waited for (its cmd_done) before the next is sent, so the judge can place
each line's frames between its cmd and cmd_done lines. A step that could not be carried out is
INCONCLUSIVE (Proc.command), never a pass.
"""
import gzip
import json
import os
import pathlib
import signal
import time

import pytest

import canopennode_facts as F
import cia301
import judges
from conftest import DEVICE_IF, HOST_IF, Proc, inconclusive
from test_master_processes import CLI_BIN, run_to_end

NODE = F.NODE_ID
TPDO1 = cia301.TPDO_BASE[1] + NODE
RPDO1 = cia301.RPDO_BASE[1] + NODE
LOOP = F.LOOPBACK_OBJECT
TPDO_MAP = [(LOOP[0], 1, 32), (LOOP[0], 2, 32)]  # N1, N4: 8 bytes.
RPDO_MAP = [(LOOP[0], 1, 16)]  # N4, N6: 2 bytes.
N4_VALUES = (F.LOOPBACK_DISABLED | 0x123, F.LOOPBACK_DISABLED | 0x456)
N4_RPDO = bytes.fromhex("2301")  # 0x1280:01 = 0x00000123 (bits 11..29 clear: accepted).
N5_VALUES = 24
N5_PAD_EVERY = 3  # A SYNC with no RPDO before value i when i % 3 == 1: both parities occur.
N5_MAX_ATTEMPTS = 3  # The judge fails a value not applied by the second SYNC; the third shows it.


class Rig:
    """Both captures, the proxy (no rules), canopend processes and our master's console."""

    def __init__(self, row, capture, proxy, start_canopend, procs):
        self.row = row
        self.host, self.device = capture(HOST_IF), capture(DEVICE_IF)
        self.proxy, self.start_canopend, self.procs = proxy, start_canopend, procs
        self.daemons = []
        self.cli = None
        self.harness = {}

    def canopend(self, **kw):
        d = self.start_canopend(**kw)
        self.daemons.append(d)
        return d

    def console(self, nodes=str(NODE)):
        assert CLI_BIN.exists(), f"NOT_RUN: {CLI_BIN} not built"
        p = Proc("canopen_master_cli", [CLI_BIN, "--if", HOST_IF, "--nodes", nodes,
                                        "--scenario", "console"], stdin=True)
        self.procs.append(p)
        p.wait_for(lambda e: e.get("event") == "start", 2, "start line")
        self.cli = p
        return p

    def run(self, line, timeout=5):
        """One console line, carried out; returns its number n."""
        return self.cli.command(line, timeout)["n"]

    def sdo(self, line):
        """An SDO console line; returns (n, its sdo_* event)."""
        n = self.run(line)
        events = list(self.cli.events)
        return n, judges.cn_sdo({"cli": events}, n)

    def read(self, index, sub, kind="u32"):
        n, e = self.sdo(f"read {NODE} {index:#x} {sub} {kind}")
        if e["event"] != "sdo_ok":
            inconclusive(f"upload {index:#06x}:{sub}: {e}")
        return n, e["value"]

    def write(self, index, sub, kind, value):
        n, e = self.sdo(f"write {NODE} {index:#x} {sub} {kind} {value:#x}")
        if e["event"] != "sdo_ok":
            inconclusive(f"download {index:#06x}:{sub} = {value:#x}: {e}")
        return n

    def nmt(self, cs):
        n = self.run(f"nmt {NODE} {cs:#x}")
        self.harness.setdefault("nmt", []).append([cs, n])
        return n

    def index_of(self, n):
        return next(i for i, e in enumerate(self.cli.events)
                    if e.get("event") == "cmd" and e.get("n") == n)

    def wait_after(self, n, kind, timeout=2.0, **fields):
        """The first master event of `kind` printed after console line n."""
        i = self.index_of(n)
        return self.cli.wait_for(
            lambda e: e.get("event") == kind and all(e.get(k) == v for k, v in fields.items())
            and self.cli.events.index(e) > i, timeout, f"{kind} {fields} after line {n}")

    def wait_state(self, n, state, timeout=2.0):
        return self.wait_after(n, "state", timeout, node=NODE, state=state)

    def configure_pdo(self, comm, mapping, cob, entries, ttype):
        """CiA 301 dynamic mapping by expedited SDO: disable (bit 31), sub0 = 0, the entries,
        sub0 = n, transmission type, enable. Returns the console numbers."""
        lines = [self.write(comm, 1, "u32", 0x80000000 | cob), self.write(mapping, 0, "u8", 0)]
        for i, (index, sub, bits) in enumerate(entries, 1):
            lines.append(self.write(mapping, i, "u32", (index << 16) | (sub << 8) | bits))
        lines += [self.write(mapping, 0, "u8", len(entries)), self.write(comm, 2, "u8", ttype),
                  self.write(comm, 1, "u32", cob)]
        return lines

    def step_sync(self):
        """`sync step`, then the SYNC it produced and node 2's TPDO1 of that SYNC."""
        done = len(self.cli.of("sync"))
        n = self.run("sync step")
        self.cli.wait_for(lambda e: e.get("event") == "sync" and e["sync_index"] == done + 1, 2,
                          f"SYNC {done + 1}")
        self.cli.wait_for(lambda e: e.get("event") == "pdo" and e.get("cob") == TPDO1 and
                          e["sync_index"] == done + 1, 2, f"TPDO1 of SYNC {done + 1}")
        return n

    def end(self, cli_rc=None):
        """Ends the console (quit), stops canopend and the proxy; returns the observation."""
        if self.cli is not None and self.cli.proc.stdin is not None:
            self.cli.send("quit")
            rc, _ = run_to_end(self.cli, 10)
            self.harness["cli_rc"] = rc
        elif cli_rc is not None:
            self.harness["cli_rc"] = cli_rc
        time.sleep(0.05)
        host, device = self.host.stop(), self.device.stop()
        for d in self.daemons:
            d.stop()
        return {"row": self.row,
                "host": [[t, c, d.hex()] for t, c, d in host],
                "device": [[t, c, d.hex()] for t, c, d in device],
                "cli": [{k: v for k, v in e.items() if k != "t_rx"} for e in self.cli.events],
                "canopend": [d.record() for d in self.daemons],
                "proxy_exit": self.proxy.stop(),
                "harness": self.harness}


def record(obs, name=None):
    record_dir = os.environ.get("INTEROP_RECORD_DIR")
    if record_dir:
        path = pathlib.Path(record_dir) / f"{name or obs['row'].lower()}.json.gz"
        path.parent.mkdir(parents=True, exist_ok=True)
        with gzip.open(path, "wt", encoding="utf-8") as f:
            json.dump(obs, f, separators=(",", ":"))


def judge(obs, name=None):
    """Judges the observation; with INTEROP_RECORD_DIR set, records it there first."""
    record(obs, name)
    status, text = judges.verdict(obs)
    print(f"{obs['row']} {status}: {text}")
    if status == "INCONCLUSIVE":
        inconclusive(text)
    assert status == "PASS", f"{obs['row']} {status}: {text}"


@pytest.fixture
def rig(capture, proxy, start_canopend, procs):
    return lambda row: Rig(row, capture, proxy, start_canopend, procs)


def booted(rig):
    """Our master's console, then canopend node 2 (so the master sees its boot-up), in
    Pre-operational. Returns the canopend process."""
    rig.console()
    n = rig.run("sync resume")  # A no-op on a running SYNC; a line to wait after.
    d = rig.canopend()
    rig.wait_state(n, cia301.STATE_PRE_OPERATIONAL)
    return d


def test_master_nmt_states_and_service_gating(rig):
    """N1. Our master sends Start, Stop, Enter Pre-operational, Reset Communication and Reset
    Node to node 2 while it produces SYNC every 10 ms. After each, canopend's heartbeat byte is
    the CiA 301 state, our master's `state` event agrees and canopend logs the same state. Reset
    Communication: boot-up, then 0x7F. Reset Node: in this build the application reset ends the
    canopend process (exit 0, Task 0), the harness restarts it on the same storage, boot-up
    follows; the exit, the restart and the boot-up are recorded separately. Service gating:
    Pre-operational answers SDO and sends no TPDO at SYNC; Operational sends TPDO1 (the 8-byte
    N4 map, type 1) after every SYNC; Stopped leaves our SDO upload of 0x1000 unanswered (no
    0x582 frame) and our master reports sdo_timeout."""
    r = rig("N1")
    first = booted(r)
    r.harness["config"] = r.configure_pdo(0x1800, 0x1A00, TPDO1, TPDO_MAP, 1)
    r.run(f"rxpdo {NODE} {TPDO1:#x} " + " ".join(f"{i:#x}:{s}:{b}" for i, s, b in TPDO_MAP))
    sdo = r.harness["sdo"] = []
    time.sleep(0.1)
    sdo.append([cia301.STATE_PRE_OPERATIONAL, r.read(0x1000, 0)[0]])

    n = r.nmt(cia301.NMT_START)
    r.wait_state(n, cia301.STATE_OPERATIONAL)
    time.sleep(0.15)
    sdo.append([cia301.STATE_OPERATIONAL, r.read(0x1000, 0)[0]])

    n = r.nmt(cia301.NMT_STOP)
    r.wait_state(n, cia301.STATE_STOPPED)
    time.sleep(0.05)
    sdo.append([cia301.STATE_STOPPED, r.sdo(f"read {NODE} 0x1000 0 u32")[0]])
    time.sleep(0.1)

    n = r.nmt(cia301.NMT_ENTER_PRE_OPERATIONAL)
    r.wait_state(n, cia301.STATE_PRE_OPERATIONAL)
    time.sleep(0.1)
    sdo.append([cia301.STATE_PRE_OPERATIONAL, r.read(0x1000, 0)[0]])

    n = r.nmt(cia301.NMT_RESET_COMMUNICATION)
    r.wait_after(n, "boot", node=NODE)
    r.wait_state(n, cia301.STATE_PRE_OPERATIONAL)
    time.sleep(0.15)

    n = r.nmt(cia301.NMT_RESET_NODE)
    rc = first.wait_exit(3.0)
    if rc is None:
        inconclusive("canopend did not end at Reset Node")
    r.harness["reset_node"] = {"exit_code": rc, "exit_seen": first.exited_at}
    r.canopend(storage_dir=first.storage_dir)
    r.wait_after(n, "boot", node=NODE)
    r.wait_state(n, cia301.STATE_PRE_OPERATIONAL)
    time.sleep(0.1)
    sdo.append([cia301.STATE_PRE_OPERATIONAL, r.read(0x1000, 0)[0]])
    judge(r.end())


def test_master_first_heartbeat_missing_vs_running_timeout(rig):
    """N2. Phase A: our master is started with node 2 expected and no slave on the bus: it
    reports boot_timeout for node 2 1000 ms +- 100 ms after its first step (its first SYNC on
    vcan_host, where C7's consumer starts) and sends no NMT. Phase B: canopend starts (0x1017 =
    100 ms from the seeded storage, read by our SDO upload), at least three heartbeats, then
    SIGSTOP: hb_lost 250-400 ms after the last heartbeat and no NMT; SIGCONT: the heartbeat
    resumes (0x7F, no boot-up) and our master reports hb_resumed, still without any NMT."""
    r = rig("N2")
    cli = r.console()
    cli.wait_for(lambda e: e.get("event") == "boot_timeout", 2.5, "boot_timeout")
    time.sleep(0.2)
    d = r.canopend()
    cli.wait_for(lambda e: e.get("event") == "boot", 2, "boot-up of node 2")
    r.harness["read_1017"] = r.read(0x1017, 0, "u16")[0]
    time.sleep(0.4)
    r.harness["sigstop"] = time.monotonic()
    d.signal(signal.SIGSTOP)
    time.sleep(0.6)
    r.harness["sigcont"] = time.monotonic()
    d.signal(signal.SIGCONT)
    t = r.harness["sigcont"]
    cli.wait_for(lambda e: e.get("event") == "hb_resumed" and e["t"] > t, 2, "hb_resumed")
    time.sleep(0.3)
    judge(r.end())


def test_master_sdo_expedited_and_aborts(rig):
    """N3. Our master uploads 0x1000 and 0x1018:01 (the example OD's 0 and 0, Task 0); with node
    2's heartbeat consumer disabled (C7) it downloads 0x1017 = 400 (canopend's heartbeat goes to
    400 ms), uploads 400, downloads 100 again, re-enables the consumer and sees at least three
    heartbeats with no hb_lost. Aborts, decoded with the codes canopend answers (pinned in Task
    0): unknown object 0x2FFF upload and download 0x06020000, download to the read-only 0x1000
    0x06010002, unknown sub-index 0x1018:05 0x06090011, one byte into the 2-byte 0x1017
    0x06070013, four bytes 0x06070012; 0x1017 still reads 100 after them."""
    r = rig("N3")
    booted(r)
    h = r.harness
    h["device_type"] = r.read(0x1000, 0)[0]
    h["vendor"] = r.read(0x1018, 1)[0]
    h["consumer_off"] = r.run(f"hb {NODE} 0")
    h["write_400"] = r.write(0x1017, 0, "u16", 400)
    h["read_400"] = r.read(0x1017, 0, "u16")[0]
    time.sleep(1.0)
    h["write_100"] = r.write(0x1017, 0, "u16", 100)
    h["read_100"] = r.read(0x1017, 0, "u16")[0]
    h["consumer_on"] = r.run(f"hb {NODE} 250")
    time.sleep(0.5)
    h["aborts"] = {}
    for label, (verb, index, sub, _) in judges.CN_ABORTS.items():
        kind = {"too_short_download": "u8", "too_long_download": "u32"}.get(label, "u32")
        line = f"read {NODE} {index:#x} {sub} {kind}" if verb == "read" else \
            f"write {NODE} {index:#x} {sub} {kind} 1"
        h["aborts"][label] = r.sdo(line)[0]
    h["read_after_aborts"] = r.read(0x1017, 0, "u16")[0]
    time.sleep(0.1)
    judge(r.end())


def run_pdo_rows(r):
    """N4/N6: TPDO1 with the 8-byte map (type 1) and RPDO1 with the 2-byte map (type 255) by
    expedited SDO; loopback values read back by SDO; Operational with SYNC running; then RPDO1
    of DLC 2, DLC 1 (too short) and DLC 2 again, each from our master's console."""
    booted(r)
    h = r.harness
    h["tpdo_map"], h["rpdo_map"] = TPDO_MAP, RPDO_MAP
    h["config"] = [r.write(LOOP[0], 1, "u32", N4_VALUES[0]), r.write(LOOP[0], 2, "u32", N4_VALUES[1])]
    h["config"] += r.configure_pdo(0x1800, 0x1A00, TPDO1, TPDO_MAP, 1)
    h["config"] += r.configure_pdo(0x1400, 0x1600, RPDO1, RPDO_MAP, 255)
    r.run(f"rxpdo {NODE} {TPDO1:#x} " + " ".join(f"{i:#x}:{s}:{b}" for i, s, b in TPDO_MAP))
    h["read_1"] = r.read(LOOP[0], 1)[0]
    h["read_2"] = r.read(LOOP[0], 2)[0]
    n = r.nmt(cia301.NMT_START)
    r.wait_state(n, cia301.STATE_OPERATIONAL)
    time.sleep(0.15)
    r.run(f"send {RPDO1:#x} {N4_RPDO.hex()}")
    time.sleep(0.02)
    h["read_rpdo"] = r.read(LOOP[0], 1)[0]
    time.sleep(0.15)
    r.run(f"send {RPDO1:#x} {N4_RPDO[:1].hex()}")
    time.sleep(0.15)
    r.run(f"send {RPDO1:#x} {N4_RPDO.hex()}")
    time.sleep(0.15)
    return r.end()


def test_master_pdo_bytes_and_lengths(rig):
    """N4. Our master maps canopend's TPDO1 to 0x1280:01 + 0x1280:02 (8 bytes, type 1) and RPDO1
    to 16 bits of 0x1280:01 (2 bytes, type 255) with the CiA 301 dynamic mapping sequence (the
    only mappable objects of the example OD, Task 0). In Operational every TPDO1 on vcan_host has
    DLC 8 and the bytes of the values our master read back by SDO, and our master's pdo event of
    the same SYNC decodes the same values; our 2-byte RPDO1 reaches vcan_device with DLC 2 and is
    applied (SDO and the next TPDOs show it); a DLC 1 RPDO1 makes canopend send its PDO-length
    EMCY 0x8210 (pinned frame) without leaving Operational."""
    judge(run_pdo_rows(rig("N4")))


def test_master_emcy_reception(rig):
    """N6. The N4 sequence again: canopend's PDO-length EMCY 0x8210 on 0x082 (after the DLC 1
    RPDO1) and the error-reset EMCY 0x0000 it sends after the next RPDO1 of the right length
    (Task 0) are each decoded by our master's emcy event with the frame's code and error
    register."""
    judge(run_pdo_rows(rig("N6")))


def run_sync_order(r, rpdo_type):
    """N5: SYNC paused before canopend starts (so every SYNC canopend saw is on vcan_device after
    its boot-up); RPDO1 and TPDO1 map 0x1280:01 (32 bits), TPDO1 type 1; Operational. For each
    value k: an optional SYNC with nothing pending, then RPDO1 = 0x80000000 | k, an SDO upload of
    0x1280:01, one SYNC step, the upload again; until k is visible the RPDO1 is sent again
    before the next step (a synchronous RPDO is written every cycle)."""
    cli = r.console()
    n = r.run("sync pause")
    r.canopend()
    r.wait_state(n, cia301.STATE_PRE_OPERATIONAL)
    h = r.harness
    h["rpdo_type"] = rpdo_type
    h["config"] = [r.write(LOOP[0], 1, "u32", F.LOOPBACK_DISABLED)]
    h["config"] += r.configure_pdo(0x1400, 0x1600, RPDO1, [(LOOP[0], 1, 32)], rpdo_type)
    h["config"] += r.configure_pdo(0x1800, 0x1A00, TPDO1, [(LOOP[0], 1, 32)], 1)
    r.run(f"rxpdo {NODE} {TPDO1:#x} {LOOP[0]:#x}:1:32")
    h["initial"] = r.read(*LOOP)[0]
    n = r.nmt(cia301.NMT_START)
    r.wait_state(n, cia301.STATE_OPERATIONAL)
    h["values"] = []
    for i in range(N5_VALUES):
        k = 0x100 + 7 * i
        value = F.LOOPBACK_DISABLED | k
        entry = {"k": k, "pads": [], "attempts": []}
        if i % N5_PAD_EVERY == 1:
            entry["pads"].append(r.step_sync())
        for _ in range(N5_MAX_ATTEMPTS):
            a = {"rpdo": r.run(f"send {RPDO1:#x} {value.to_bytes(4, 'little').hex()}"),
                 "before": r.read(*LOOP)[0], "step": r.step_sync()}
            a["after"], seen = r.read(*LOOP)
            entry["attempts"].append(a)
            if seen == value:
                break
        h["values"].append(entry)
    assert cli.proc.poll() is None
    return r.end()


def test_master_sync_application_and_sampling_order(rig):
    """N5. With a synchronous RPDO1 (type 0) our master pauses SYNC and steps it one at a time:
    k is never visible before the first SYNC after its RPDO; it is visible by SDO and in the
    TPDO1 our master decodes after the SYNC that applied it; that SYNC is the first or the second
    after the RPDO, as the pinned CANopenNode 76b43c8 parity defect (CO_PDO.c:811, Task 0's
    SYNC_RPDO_SEQUENCE_AFTER_EACH_SYNC) predicts from the SYNCs canopend has seen since boot-up, so
    a wrong SYNC still fails; 24 values, both parities. Then the control: the same sequence with
    RPDO1 type 255 (applied on reception) must FAIL the same judge (recorded as n5_type255, the
    committed negative fixture)."""
    judge(run_sync_order(rig("N5"), 0))


def test_master_sync_order_judge_rejects_type_255(rig):
    """N5's control on the wire: the N5 sequence with RPDO1 type 255 is judged FAIL (k visible
    before the SYNC), so the N5 verdict is not blind to an RPDO applied on reception."""
    obs = run_sync_order(rig("N5"), 255)
    record(obs, "n5_type255")
    status, text = judges.verdict(obs)
    print(f"N5 (type 255) {status}: {text}")
    if status == "INCONCLUSIVE":
        inconclusive(text)
    assert status == "FAIL" and "visible by SDO before" in text, f"{status}: {text}"


def test_master_refuses_activation_on_canopennode(rig):
    """N7. Six canopend processes as nodes 1-6 (activate_hal's session is fixed at six nodes;
    each is a generic node: zero identity, no 0x2012). `canopen_master_cli --scenario activate_hal
    --required all` configures (0x2012 aborts 0x06020000 on every node, so the profile is
    generic_cia402_profile()) and activation is refused listing all five capabilities:
    device_side_expiry, execution_generation, apply_ack unsupported, local_stop,
    source_timestamp unknown. Neither link shows an NMT Start or an RPDO."""
    r = rig("N7")
    for node in judges.CN_SESSION_NODES:
        r.canopend(node_id=node)
    time.sleep(0.3)
    assert CLI_BIN.exists(), f"NOT_RUN: {CLI_BIN} not built"
    cli = Proc("canopen_master_cli", [CLI_BIN, "--if", HOST_IF, "--nodes", "1-6",
                                      "--scenario", "activate_hal", "--required", "all"])
    r.procs.append(cli)
    r.cli = cli
    rc, _ = run_to_end(cli, 20)
    time.sleep(0.3)
    judge(r.end(cli_rc=rc))
