# SPDX-License-Identifier: Apache-2.0
"""Calibration of the independent CANopenNode slave (canopend, E3c Task 0) before the master
interop matrix (N1-N7) depends on it, in the manner of test_peer_config.py for Lely.

The slave is CANopenLinux v4.0 + CANopenNode 76b43c8 as built into the dev image (no self-start
to Operational, log on stderr; docker/canopennode_build.h), node 2, the shipped example DS301
object dictionary, storage pre-seeded by canopend.seed_storage (0x1017 = 100 ms). Each test
starts its own canopend on vcan_device directly (no proxy, no master) and talks to it with the
raw client in canopend.py. The constants are in canopennode_facts.py.

Observed (the first column is the test that pins it):

  does_not_self_start     boot-up 0x00 on 0x702, then 0x7F every 100 ms; never 0x05
  first_start_...         seeded storage: no frame on 0x082 in the first second. Without storage
                          files: EMCY 0x5000 and no heartbeat at all (shipped 0x1017 = 0)
  reports_nmt_state       log line `<5>CANopen NMT state changed to: "<state>" (<n>)` for each of
                          initializing, pre-operational, operational, stopped, in step with the
                          heartbeat byte; the log is the only report canopend has of its state
  example_od_facts        0x1000 = 0, 0x1018:01..04 = 0 (the shipped OD has no identity), 0x1017 =
                          100 seeded (0 shipped), 0x1005 = 0x80, 0x1014 = 0x82
  sdo_abort_codes         0x06020000 unknown object, 0x06090011 unknown sub-index, 0x06010002
                          write to read-only, 0x06070013 / 0x06070012 wrong download size
  pdo_configuration       communication and mapping objects are writable by SDO (CiA 301 dynamic
                          mapping); the example OD maps only 0x1001, 0x1200:1/2 (TPDO), 0x1280:1/2
  pdo_length_emcy         0x8210 (RPDO too short), 0x8220 (too long), 0x0000 when it clears
  service_gating          Stopped: no SDO answer; Pre-operational: SDO, no TPDO at SYNC;
                          Operational: TPDO at SYNC
  reset_node_behaviour    process exits 0 and puts nothing on the bus; restart gives boot-up
  sync_rpdo_...           applied at SYNC for types 0-240, at reception for 255; defect: every
                          second SYNC does not apply it (see canopennode_facts.py)
"""
import collections
import pathlib
import re
import shutil
import signal
import struct
import time

import pytest

import canopend as co
import canopennode_facts as F
import cia301
from conftest import DEVICE_IF, ROOT, inconclusive

HB = cia301.HEARTBEAT_BASE + F.NODE_ID
EMCY = F.EMCY_COB_ID_1014
SDO_TX = cia301.SDO_TX_BASE + F.NODE_ID
TPDO1 = cia301.TPDO_BASE[1] + F.NODE_ID
RPDO1 = cia301.RPDO_BASE[1] + F.NODE_ID
LOOP = F.LOOPBACK_OBJECT


@pytest.fixture(scope="session")
def seed_dir(vcan, tmp_path_factory):
    assert co.BIN.exists() or shutil.which(str(co.BIN)), \
        f"NOT_RUN: canopend ({co.BIN}) not found; build the dev image"
    try:
        return co.seed_storage(tmp_path_factory.mktemp("canopend_seed"))
    except (RuntimeError, OSError, co.SdoAbort, co.SdoTimeout) as e:
        pytest.fail(f"NOT_RUN: cannot seed the canopend storage: {e}", pytrace=False)


@pytest.fixture
def start_canopend(vcan, seed_dir, tmp_path):
    """start(seeded=True) -> Canopend, ready, with its own copy of the storage."""
    started, counter = [], [0]

    def start(seeded=True, storage_dir=None):
        if storage_dir is None:
            counter[0] += 1
            storage_dir = tmp_path / f"store{counter[0]}"
            if seeded:
                shutil.copytree(seed_dir, storage_dir)
            else:
                storage_dir.mkdir()
        d = co.Canopend(DEVICE_IF, storage_dir)
        started.append(d)
        if not co.ready(d):
            inconclusive(f"canopend did not report running: {d.lines()[-5:]}; exit {d.proc.poll()}")
        return d
    yield start
    for d in started:
        d.stop()


@pytest.fixture
def bus(vcan):
    b = co.Bus(DEVICE_IF)
    yield b
    b.close()


def of(frames, cob, since=0.0):
    return [(t, d) for t, c, d in frames if c == cob and t >= since]


def states(frames, since=0.0):
    return [d[0] for t, d in of(frames, HB, since)]


def settle(seconds):
    time.sleep(seconds)


def sdo_code(bus, index, sub):
    """Upload abort code, the value, or the string 'timeout'."""
    try:
        return bus.sdo_upload_int(F.NODE_ID, index, sub)
    except co.SdoAbort as e:
        return ("abort", e.code)
    except co.SdoTimeout:
        return "timeout"


def dl(bus, index, sub, value, size):
    """'ok', ('abort', code) or 'timeout'."""
    try:
        bus.sdo_download(F.NODE_ID, index, sub, value, size)
        return "ok"
    except co.SdoAbort as e:
        return ("abort", e.code)
    except co.SdoTimeout:
        return "timeout"


def configure_tpdo1(bus, mapping, ttype=1):
    """CiA 301 dynamic mapping of TPDO1: disable, sub0 = 0, entries, sub0 = n, type, enable."""
    results = [dl(bus, 0x1800, 1, 0x80000000 | TPDO1, 4), dl(bus, 0x1A00, 0, 0, 1)]
    for i, (index, sub, bits) in enumerate(mapping, 1):
        results.append(dl(bus, 0x1A00, i, (index << 16) | (sub << 8) | bits, 4))
    results += [dl(bus, 0x1A00, 0, len(mapping), 1), dl(bus, 0x1800, 2, ttype, 1),
                dl(bus, 0x1800, 1, TPDO1, 4)]
    assert results == ["ok"] * len(results), f"INCONCLUSIVE: TPDO1 configuration {results}"


def configure_rpdo1(bus, mapping, ttype):
    results = [dl(bus, 0x1400, 1, 0x80000000 | RPDO1, 4), dl(bus, 0x1600, 0, 0, 1)]
    for i, (index, sub, bits) in enumerate(mapping, 1):
        results.append(dl(bus, 0x1600, i, (index << 16) | (sub << 8) | bits, 4))
    results += [dl(bus, 0x1600, 0, len(mapping), 1), dl(bus, 0x1400, 2, ttype, 1),
                dl(bus, 0x1400, 1, RPDO1, 4)]
    assert results == ["ok"] * len(results), f"INCONCLUSIVE: RPDO1 configuration {results}"


def test_canopend_build_matches_the_pinned_commits():
    """The Dockerfile ARGs, the facts and the commits recorded in the image agree, and
    THIRD_PARTY_NOTICES names them. A change of the pin must change all of them together."""
    dockerfile = (ROOT / "docker" / "Dockerfile").read_text()
    for name, value in (("CANOPENLINUX_COMMIT", F.CANOPENLINUX_COMMIT),
                        ("CANOPENNODE_COMMIT", F.CANOPENNODE_COMMIT),
                        ("CANOPENLINUX_SHA256", F.CANOPENLINUX_SHA256),
                        ("CANOPENNODE_SHA256", F.CANOPENNODE_SHA256)):
        assert re.search(rf"^ARG {name}={value}$", dockerfile, re.M), f"Dockerfile {name}"
    notices = (ROOT / "THIRD_PARTY_NOTICES").read_text()
    for value in (F.CANOPENLINUX_COMMIT, F.CANOPENNODE_COMMIT, F.CANOPENLINUX_SHA256,
                  F.CANOPENNODE_SHA256):
        assert value in notices, f"THIRD_PARTY_NOTICES lacks {value}"
    recorded = pathlib.Path("/opt/canopennode/COMMITS")
    assert recorded.exists(), "NOT_RUN: /opt/canopennode/COMMITS missing; build the dev image"
    assert recorded.read_text().split() == [F.CANOPENLINUX_COMMIT, F.CANOPENNODE_COMMIT]
    assert co.BIN.exists() or shutil.which(str(co.BIN)), "NOT_RUN: canopend not found"


def test_canopend_does_not_self_start(start_canopend, capture):
    """Stock canopend starts itself to Operational (NMT_CONTROL has CO_NMT_STARTUP_TO_OPERATIONAL,
    gate check 3). The pinned build lacks that bit: after boot-up 0x00 the heartbeat is 0x7F for
    as long as nobody sends NMT Start, and 0x05 never appears. Heartbeat period is 100 ms (the
    seeded 0x1017)."""
    cap = capture(DEVICE_IF)
    start_canopend()
    settle(1.3)
    frames = cap.stop()
    seen = states(frames, 0)
    assert seen and seen[0] == cia301.STATE_BOOT_UP, f"INCONCLUSIVE: no boot-up first: {seen[:3]}"
    after = seen[1:]
    beats = of(frames, HB)[1:]
    assert beats[-1][0] - beats[0][0] >= 1.0, "INCONCLUSIVE: heartbeat observed for less than 1 s"
    assert set(after) == {cia301.STATE_PRE_OPERATIONAL}, seen
    assert cia301.STATE_OPERATIONAL not in seen
    gaps = [b[0] - a[0] for a, b in zip(beats, beats[1:])]
    assert all(abs(g - 0.1) < F.HEARTBEAT_TOLERANCE_S + 0.02 for g in gaps), gaps


def test_canopend_first_start_emits_no_storage_emcy(start_canopend, capture):
    """With the pre-seeded storage a first start sends no EMCY (no frame on 0x082 within 1 s of
    boot-up), the property N6 relies on to see only the EMCY it provokes."""
    cap = capture(DEVICE_IF)
    start_canopend()
    settle(1.1)
    frames = cap.stop()
    assert of(frames, HB), "INCONCLUSIVE: no heartbeat seen, the capture saw nothing"
    assert of(frames, EMCY) == []


def test_canopend_without_storage_raises_0x5000_and_has_no_heartbeat(start_canopend, capture):
    """The control of the two tests above (so that 'no EMCY' cannot be a blind detector): without
    storage files canopend sends EMCY 0x5000 (non-volatile memory) once at start, and the shipped
    0x1017 = 0 means no periodic heartbeat after the boot-up frame."""
    cap = capture(DEVICE_IF)
    d = start_canopend(seeded=False)
    settle(0.6)
    frames = cap.stop()
    emcy = of(frames, EMCY)
    assert emcy, f"INCONCLUSIVE: no EMCY at all; log {d.lines()}"
    assert struct.unpack_from("<H", emcy[0][1])[0] == F.EMCY_NON_VOLATILE_MEMORY, emcy[0][1].hex()
    assert states(frames) == [cia301.STATE_BOOT_UP], states(frames)


def test_canopend_reports_nmt_state(start_canopend, capture, bus):
    """canopend logs every NMT state change on stderr as `<5>CANopen NMT state changed to:
    "<state>" (<n>)`. On start: initializing then pre-operational; then Start, Stop, Enter
    Pre-operational, Start each add the matching line, and the heartbeat byte agrees."""
    cap = capture(DEVICE_IF)
    d = start_canopend()
    settle(0.15)
    start_lines = [line for line in d.lines() if "NMT state" in line]
    assert start_lines == [F.LOG_NMT["initializing"], F.LOG_NMT["pre_operational"]], start_lines
    expected = ((cia301.NMT_START, "operational", cia301.STATE_OPERATIONAL),
                (cia301.NMT_STOP, "stopped", cia301.STATE_STOPPED),
                (cia301.NMT_ENTER_PRE_OPERATIONAL, "pre_operational", cia301.STATE_PRE_OPERATIONAL),
                (cia301.NMT_START, "operational", cia301.STATE_OPERATIONAL))
    for command, name, byte in expected:
        before = len([line for line in d.lines() if "NMT state" in line])
        bus.nmt(command, F.NODE_ID)
        assert d.wait_log(F.LOG_NMT[name], 1.0) is not None, \
            f"no log line for {name}: {d.lines()[-3:]}"
        settle(0.25)
        now = [line for line in d.lines() if "NMT state" in line]
        assert now[before:] == [F.LOG_NMT[name]], now[before:]
        assert states(list(cap.frames))[-1] == byte, (name, states(list(cap.frames))[-4:])
    cap.stop()


def test_canopend_example_od_facts(start_canopend, bus):
    """The shipped example OD (CANopenNode example/OD.c, 'DS301 profile') read by SDO upload:
    0x1000 = 0, 0x1018 has 4 sub-indices all 0 (no vendor, product, revision, serial: N3 can
    only pin the zeros), 0x1005 = 0x80, 0x1014 = 0x82. 0x1017 reads 100 from the seeded storage;
    the shipped value (0) is read from an unseeded start. The image carries OD.c: the same
    values must be in it."""
    start_canopend()
    assert sdo_code(bus, 0x1000, 0) == F.DEVICE_TYPE
    assert sdo_code(bus, 0x1018, 0) == F.IDENTITY_HIGHEST_SUB
    for sub, value in F.IDENTITY.items():
        assert sdo_code(bus, 0x1018, sub) == value, sub
    assert sdo_code(bus, 0x1017, 0) == F.SEEDED_HEARTBEAT_MS
    assert sdo_code(bus, 0x1005, 0) == F.SYNC_COB_ID_1005
    assert sdo_code(bus, 0x1014, 0) == F.EMCY_COB_ID_1014
    od = pathlib.Path("/opt/canopennode/OD.c").read_text()
    assert ".x1017_producerHeartbeatTime = 0x0000," in od, "shipped 0x1017 is not 0"
    assert ".x1000_deviceType = 0x00000000," in od
    assert ".vendor_ID = 0x00000000," in od


def test_canopend_shipped_heartbeat_time_is_zero(start_canopend, bus):
    """An unseeded canopend reads 0x1017 = 0 (SHIPPED_HEARTBEAT_MS)."""
    start_canopend(seeded=False)
    assert sdo_code(bus, 0x1017, 0) == F.SHIPPED_HEARTBEAT_MS


def test_canopend_heartbeat_time_is_writable(start_canopend, bus, capture):
    """0x1017 is read/write 16 bit by SDO (N3 rewrites it): 400 reads back 400 and the period
    becomes 400 ms, 100 puts it back. Not persisted unless saved (0x1010)."""
    cap = capture(DEVICE_IF)
    start_canopend()
    assert dl(bus, 0x1017, 0, 400, 2) == "ok"
    assert sdo_code(bus, 0x1017, 0) == 400
    mark = time.monotonic()
    settle(1.3)
    assert dl(bus, 0x1017, 0, 100, 2) == "ok"
    assert sdo_code(bus, 0x1017, 0) == 100
    settle(0.4)
    frames = cap.stop()
    times = [t for t, d in of(frames, HB) if t >= mark]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert any(abs(g - 0.4) < 0.05 for g in gaps), gaps
    assert any(abs(g - 0.1) < 0.05 for g in gaps[-3:]), gaps


def test_canopend_sdo_abort_codes(start_canopend, bus):
    """The abort codes canopend answers with (the codes N3 pins): unknown object 0x2FFF
    0x06020000 (upload and download), unknown sub-index 0x1018:05 0x06090011, download to the
    read-only 0x1000 and 0x1018:01 0x06010002, wrong sizes into the 2-byte 0x1017: 1 byte
    0x06070013 (too low), 4 bytes 0x06070012 (too high); 1 byte into the 4-byte 0x1280:01
    0x06070013; 0x1280:01 = 0x1234 (bits 11..29 set) 0x06090030."""
    start_canopend()
    assert sdo_code(bus, 0x2FFF, 0) == ("abort", F.ABORT_NO_OBJECT)
    assert dl(bus, 0x2FFF, 0, 1, 1) == ("abort", F.ABORT_NO_OBJECT)
    assert sdo_code(bus, 0x1018, 5) == ("abort", F.ABORT_NO_SUBINDEX)
    assert dl(bus, 0x1000, 0, 1, 4) == ("abort", F.ABORT_READ_ONLY)
    assert dl(bus, 0x1018, 1, 1, 4) == ("abort", F.ABORT_READ_ONLY)
    assert dl(bus, 0x1017, 0, 1, 1) == ("abort", F.ABORT_LENGTH_TOO_LOW)
    assert dl(bus, 0x1017, 0, 1, 4) == ("abort", F.ABORT_LENGTH_TOO_HIGH)
    assert dl(bus, 0x1280, 1, 1, 1) == ("abort", F.ABORT_LENGTH_TOO_LOW)
    assert dl(bus, *LOOP, 0x1234, 4) == ("abort", F.ABORT_INVALID_VALUE)  # bits 11..29 set
    assert dl(bus, *LOOP, F.LOOPBACK_DISABLED | 0x7FF, 4) == "ok"
    assert sdo_code(bus, 0x1017, 0) == F.SEEDED_HEARTBEAT_MS, "a rejected write changed the value"


def test_canopend_pdo_configuration_facts(start_canopend, bus):
    """PDO configuration by SDO, in Pre-operational. Defaults: RPDO1 COB 0x80000202, TPDO1
    0xC0000182 (disabled), type 0xFE, no mapping. The communication and mapping objects are
    writable by SDO (disable, sub0 = 0, entries, sub0 = n, enable), RPDO and TPDO alike. The
    example OD has no application objects, so the mappable set is small: RPDO 0x1001:0 and
    0x1280:1/2, TPDO also 0x1200:1/2 (read-only constants); every other object is refused with
    0x06040041, "object cannot be mapped" (found by trying each sub-index of each object in the OD with its own size).
    Mapping a 64-bit entry onto a 32-bit object is refused with the same 0x06040041. TPDO1 mapped to
    0x1280:1 + 0x1280:2 (8 bytes) is accepted."""
    start_canopend()
    assert sdo_code(bus, 0x1400, 1) == F.DEFAULT_RPDO1_COB
    assert sdo_code(bus, 0x1800, 1) == F.DEFAULT_TPDO1_COB
    assert sdo_code(bus, 0x1400, 2) == F.DEFAULT_PDO_TYPE
    assert sdo_code(bus, 0x1800, 2) == F.DEFAULT_PDO_TYPE
    assert sdo_code(bus, 0x1600, 0) == 0 and sdo_code(bus, 0x1A00, 0) == 0

    sizes = {}
    for index in (0x1000, 0x1001, 0x1003, 0x1005, 0x1006, 0x1007, 0x1010, 0x1011, 0x1012,
                  0x1014, 0x1015, 0x1016, 0x1017, 0x1018, 0x1019, 0x1200, 0x1280, 0x1400,
                  0x1600, 0x1800, 0x1A00):
        for sub in range(0, 9):
            try:
                sizes[(index, sub)] = len(bus.sdo_upload(F.NODE_ID, index, sub))
            except co.SdoAbort:
                pass
    found = {"rpdo": [], "tpdo": []}
    refused = collections.Counter()
    for kind, comm, mapping in (("rpdo", 0x1400, 0x1600), ("tpdo", 0x1800, 0x1A00)):
        for (index, sub), size in sizes.items():
            assert dl(bus, mapping, 0, 0, 1) == "ok"
            put = dl(bus, mapping, 1, (index << 16) | (sub << 8) | (size * 8), 4)
            if put == "ok" and dl(bus, mapping, 0, 1, 1) == "ok":
                found[kind].append((index, sub, size))
            else:
                refused[put if put != "ok" else dl(bus, mapping, 0, 1, 1)] += 1
            assert dl(bus, mapping, 0, 0, 1) == "ok"
    assert tuple(sorted(found["rpdo"])) == F.RPDO_MAPPABLE, found
    assert tuple(sorted(found["tpdo"])) == F.TPDO_MAPPABLE, found
    assert set(refused) == {("abort", F.ABORT_NOT_MAPPABLE)}, refused

    assert dl(bus, 0x1600, 0, 0, 1) == "ok"
    assert dl(bus, 0x1600, 1, 0x12800140, 4) == ("abort", F.ABORT_NOT_MAPPABLE)
    configure_tpdo1(bus, [(LOOP[0], 1, 32), (LOOP[0], 2, 32)], ttype=1)
    assert sdo_code(bus, 0x1A00, 0) == 2
    assert sdo_code(bus, 0x1800, 1) == TPDO1


def test_canopend_service_gating_by_nmt_state(start_canopend, bus, capture):
    """What each NMT state serves (N1). Pre-operational: SDO answers, a SYNC draws no TPDO.
    Operational: SYNC draws the TPDO (DLC 8, bytes = little-endian 0x1280:01 then :02, here
    0x80000123 and 0x80000000 as written by SDO). Stopped: an SDO request gets no answer at all
    (no 0x582 frame) and a SYNC no TPDO."""
    cap = capture(DEVICE_IF)
    start_canopend()
    assert dl(bus, *LOOP, 0x80000123, 4) == "ok"
    configure_tpdo1(bus, [(LOOP[0], 1, 32), (LOOP[0], 2, 32)], ttype=1)
    wanted = struct.pack("<II", 0x80000123, 0x80000000)

    t = time.monotonic()
    bus.send(cia301.SYNC_COB)
    settle(0.1)
    assert sdo_code(bus, 0x1000, 0) == 0
    pre = [x for x in cap.frames if x[0] >= t]
    assert of(pre, TPDO1) == [], "TPDO in Pre-operational"

    bus.nmt(cia301.NMT_START, F.NODE_ID)
    settle(0.15)
    t = time.monotonic()
    bus.send(cia301.SYNC_COB)
    settle(0.1)
    op = [x for x in cap.frames if x[0] >= t]
    tpdo = of(op, TPDO1)
    assert len(tpdo) == 1 and tpdo[0][1] == wanted, [x[1].hex() for x in tpdo]
    assert sdo_code(bus, 0x1000, 0) == 0

    bus.nmt(cia301.NMT_STOP, F.NODE_ID)
    settle(0.15)
    t = time.monotonic()
    assert sdo_code(bus, 0x1000, 0) == "timeout"
    bus.send(cia301.SYNC_COB)
    settle(0.1)
    stopped = [x for x in cap.frames if x[0] >= t]
    assert of(stopped, SDO_TX) == [] and of(stopped, TPDO1) == []
    assert any(c == cia301.SDO_RX_BASE + F.NODE_ID for _, c, _ in stopped), \
        "INCONCLUSIVE: the SDO request was not on the bus"
    cap.stop()


def test_canopend_pdo_length_emcy(start_canopend, bus, capture):
    """RPDO1 mapped to 2 bytes (type 255). A 1-byte RPDO makes canopend emit EMCY 0x8210 on
    0x082 (8 bytes: code LE, error register 0x00, error bit 0x04, info code = mapped length 2);
    a following RPDO of the right length makes it send the error-reset EMCY 0x0000 (same bit);
    a 3-byte RPDO makes it send 0x8220 (too long). The NMT state does not change (heartbeat stays
    0x05), and the error register 0x1001 stays 0. The shipped 'DLC 1 -> 0x8210' of the plan holds
    for short; a long frame is a different code."""
    cap = capture(DEVICE_IF)
    d = start_canopend()
    configure_rpdo1(bus, [(LOOP[0], 1, 16)], 255)
    bus.nmt(cia301.NMT_START, F.NODE_ID)
    settle(0.15)

    def emcy_after(frame):
        t = time.monotonic()
        bus.send(RPDO1, frame)
        settle(0.3)
        return of([x for x in cap.frames if x[0] >= t], EMCY)

    short = emcy_after(b"\x01")
    assert len(short) == 1, short
    code, reg, bit, info = struct.unpack("<HBBI", short[0][1][:8])
    assert (code, reg, bit, info) == (F.EMCY_PDO_LENGTH_SHORT, F.EMCY_ERROR_REGISTER,
                                      F.EMCY_ERROR_BIT_RPDO_LENGTH, 2), short[0][1].hex()
    ok = emcy_after(struct.pack("<H", 0x0123))
    assert len(ok) == 1 and struct.unpack("<HBBI", ok[0][1][:8]) == \
        (F.EMCY_RESET_CODE, F.EMCY_ERROR_REGISTER, F.EMCY_ERROR_BIT_RPDO_LENGTH, 2), ok
    long = emcy_after(b"\x01\x02\x03")
    assert len(long) == 1 and struct.unpack("<H", long[0][1][:2])[0] == F.EMCY_PDO_LENGTH_LONG
    assert sdo_code(bus, 0x1001, 0) == 0
    frames = cap.stop()
    assert states(frames)[-1] == cia301.STATE_OPERATIONAL and cia301.STATE_STOPPED not in states(frames)
    assert any("errorCode=0x8210" in line for line in d.lines()), d.lines()[-4:]


def test_canopend_reset_node_behaviour(start_canopend, bus, capture):
    """Reset Node (NMT 0x81): in this build the application reset ends the canopend process
    (exit status 0; `-r` would reboot Linux and is not used). After the NMT frame canopend puts
    nothing on the bus (no boot-up, no heartbeat) and its log ends with 'finished'. Restarted on
    the same storage it sends boot-up 0x00 then 0x7F, 0x1017 reads the seeded 100 and the PDO
    configuration written by SDO is gone (PDO objects are not stored). N1 therefore judges
    Reset Node as exit + harness restart + boot-up."""
    cap = capture(DEVICE_IF)
    d = start_canopend()
    configure_tpdo1(bus, [(LOOP[0], 1, 32)], ttype=1)
    settle(0.15)
    t = time.monotonic()
    bus.nmt(cia301.NMT_RESET_NODE, F.NODE_ID)
    rc = d.wait_exit(3.0)
    assert rc == F.RESET_NODE_EXIT_CODE, f"exit {rc}; log {d.lines()[-3:]}"
    settle(0.4)
    after = [x for x in cap.frames if x[0] >= t]
    from_node = [x for x in after if x[1] in (HB, EMCY, SDO_TX, TPDO1)]
    assert from_node == [], from_node
    assert d.lines()[-1] == F.LOG_FINISHED
    t2 = time.monotonic()
    d2 = start_canopend(storage_dir=d.storage_dir)
    settle(0.35)
    assert d2.running()
    seen = states([x for x in cap.frames if x[0] >= t2])
    assert seen[:2] == [cia301.STATE_BOOT_UP, cia301.STATE_PRE_OPERATIONAL], seen
    assert sdo_code(bus, 0x1017, 0) == F.SEEDED_HEARTBEAT_MS
    assert sdo_code(bus, 0x1A00, 0) == 0 and sdo_code(bus, 0x1800, 1) == F.DEFAULT_TPDO1_COB
    cap.stop()


def test_canopend_reset_communication_behaviour(start_canopend, bus, capture):
    """Reset Communication (NMT 0x82): the process keeps running; boot-up 0x00 follows within
    0.2 s, then Pre-operational 0x7F again (also from Operational); the log shows
    initializing then pre-operational."""
    cap = capture(DEVICE_IF)
    d = start_canopend()
    bus.nmt(cia301.NMT_START, F.NODE_ID)
    settle(0.25)
    t = time.monotonic()
    bus.nmt(cia301.NMT_RESET_COMMUNICATION, F.NODE_ID)
    settle(0.5)
    seen = states([x for x in cap.frames if x[0] >= t])
    assert d.running()
    first_boot = seen.index(cia301.STATE_BOOT_UP)
    assert set(seen[first_boot + 1:]) == {cia301.STATE_PRE_OPERATIONAL}, seen
    assert [line for line in d.lines() if "NMT state" in line][-2:] == \
        [F.LOG_NMT["initializing"], F.LOG_NMT["pre_operational"]]
    cap.stop()


def test_canopend_sync_rpdo_applied_at_sync(start_canopend, bus):
    """RPDO1 (2 x... one 32-bit mapping 0x1280:01, type 0) and a paused SYNC. The value is not
    visible by SDO after the RPDO: it is applied by the SYNC. A type 255 RPDO is visible at once.
    Defect of the pinned commit (CO_PDO.c:811): from a fresh start, eight pairs RPDO v / SYNC
    (v = 1..8) leave 1, 1, 3, 3, 5, 5, 7, 7, i.e. every second SYNC does not apply the RPDO
    (the data is read from the wrong RX buffer); pinned here as SYNC_RPDO_SEQUENCE_AFTER_EACH_SYNC.
    N5 cannot expect 'one RPDO, one SYNC, value k' on every step from this build."""
    start_canopend()
    assert dl(bus, *LOOP, F.LOOPBACK_DISABLED, 4) == "ok"
    configure_rpdo1(bus, [(LOOP[0], 1, 32)], 0)
    bus.nmt(cia301.NMT_START, F.NODE_ID)
    settle(0.15)
    seen = []
    for v in range(1, 9):
        bus.send(RPDO1, struct.pack("<I", F.LOOPBACK_DISABLED | v))
        settle(0.03)
        before = sdo_code(bus, *LOOP)
        assert before != F.LOOPBACK_DISABLED | v, f"RPDO {v} applied before the SYNC"
        bus.send(cia301.SYNC_COB)
        settle(0.03)
        seen.append(sdo_code(bus, *LOOP) & 0x7FF)
    assert tuple(seen) == F.SYNC_RPDO_SEQUENCE_AFTER_EACH_SYNC, seen

    assert dl(bus, 0x1400, 1, 0x80000000 | RPDO1, 4) == "ok"
    assert dl(bus, 0x1400, 2, 255, 1) == "ok" and dl(bus, 0x1400, 1, RPDO1, 4) == "ok"
    for v in (0x31, 0x32, 0x33):
        bus.send(RPDO1, struct.pack("<I", F.LOOPBACK_DISABLED | v))
        settle(0.03)
        assert sdo_code(bus, *LOOP) == F.LOOPBACK_DISABLED | v, "type 255 RPDO not applied on reception"


def test_canopend_stop_and_continue_signals(start_canopend, bus, capture):
    """N2 stops the slave with SIGSTOP: no frame at all from node 2 while it is stopped (>= 0.5 s
    observed), and after SIGCONT the heartbeat resumes (0x7F, within 0.25 s) without a boot-up
    and an SDO answers again."""
    cap = capture(DEVICE_IF)
    d = start_canopend()
    settle(0.35)
    t_stop = time.monotonic()
    d.signal(signal.SIGSTOP)
    settle(0.6)
    t_cont = time.monotonic()
    d.signal(signal.SIGCONT)
    settle(0.3)
    assert sdo_code(bus, 0x1000, 0) == 0
    frames = cap.stop()
    during = [x for x in frames if t_stop + 0.12 <= x[0] <= t_cont]
    assert of(during, HB) == [], during
    resumed = of(frames, HB, t_cont)
    assert resumed and resumed[0][0] - t_cont < 0.25, resumed[:1]
    assert {x[1][0] for x in resumed} == {cia301.STATE_PRE_OPERATIONAL}
