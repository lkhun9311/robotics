# SPDX-License-Identifier: Apache-2.0
"""Calibration of the independent Lely master (lely_peer, liblely-coapp 2.3.5) before any interop
test depends on it (decision B15). The slaves here are Lely BasicSlaves (probe_slave) serving our
EDS plus 0x1F80 = 0x04, so they stay Pre-operational after boot-up as our drive does.

Observed on vcan, one option changed at a time (run 0 = bus.yml with only node IDs and the EDS):

  change                        observed effect
  (run 0, defaults)             000#8200 once at start; NMT Start to 1-6 ~0.1 s later; 0x1016 =
                                300 ms; no SYNC; after a heartbeat timeout NMT Reset Node (0x81)
                                to each timed-out slave, 0.25-0.3 s after SIGKILL
  start_nodes: false            no NMT Start; everything else unchanged
  heartbeat_multiplier: 2.5     0x1016 = 250 ms; timeout reported 0.249 s after SIGKILL
  sync_period: 10000            SYNC 0x080 every 10 ms (150 frames in 1.5 s)
  reset_communication: false    worse: Reset Communication sent to node-IDs 1..126 one by one
                                (126 frames, nodes 1-6 included) instead of one broadcast;
                                reverted
  mandatory: true, boot_time    no change in startup, timeout or boot-failure traffic
  boot: false (variant)         no SDO at all; reverted (M2 needs the identity upload)
  hb indication override        Lely's co_nmt_on_hb is not called for a timeout that occurs, so
  (main.cpp, PeerMaster)        no Reset Node follows; no dcfgen option suppressed it

Lely's 'boot slave' process uploads only 0x1000 (compared with 0x1F84) from each node; with no
concise DCF it downloads nothing. A slave whose heartbeat never arrives (0x1017 = 0) is reported
as boot es 'K' ("No heartbeat message received from CANopen device during start error control
service"), state 0; with reset_communication: false the same case was es 'E'.
"""
import collections
import configparser
import statistics
import time

import wire
from conftest import NODES, derived_eds

MASTER_ID = 127
NMT_START = wire.NMT_COMMANDS["start"]
NMT_RESET_COMM = wire.NMT_COMMANDS["reset_communication"]
NMT_RESET_NODE = wire.NMT_COMMANDS["reset_node"]
PRE_OPERATIONAL = wire.HEARTBEAT_STATES["pre_operational"]
BOOT_UP = wire.HEARTBEAT_STATES["boot_up"]

# B15: objects the master must never download to (all const in our EDS).
CONST_OBJECTS = {0x1017} | {base + i for base in (0x1400, 0x1600, 0x1800, 0x1A00) for i in range(3)}


def slave_cob_ids(eds):
    """COB-IDs only the slaves transmit; every other frame on vcan_host is the master's."""
    cobs = set()
    for n in NODES:
        cobs |= {wire.HEARTBEAT_BASE + n, wire.SDO_RESPONSE_BASE + n, eds.emcy_cob_id(n)}
        cobs |= {wire.pdo_cob_id(p, n) for p in eds.pdos() if p["label"].startswith("TPDO")}
    return cobs


def nmt_frames(frames):
    return [(t, d[0], d[1]) for t, cob, d in frames if cob == wire.NMT_COB and len(d) == 2]


def heartbeats(frames, node):
    return [(t, d[0]) for t, cob, d in frames if cob == wire.HEARTBEAT_BASE + node and len(d) == 1]


def sdo_requests(frames):
    """(t, node, command byte, index, sub) of every SDO request the master sent."""
    out = []
    for t, cob, d in frames:
        if wire.SDO_REQUEST_BASE < cob <= wire.SDO_REQUEST_BASE + 127 and len(d) == 8:
            out.append((t, cob - wire.SDO_REQUEST_BASE, d[0], d[1] | d[2] << 8, d[3]))
    return out


def is_download_initiate(command):
    return command >> 5 == 1  # CiA 301 ccs 1: initiate download


def fmt(frame):
    t, cob, d = frame
    return f"{t:.6f} {cob:03X}#{d.hex().upper()}"


def boot_all(peer, timeout=5):
    for n in NODES:
        peer.wait_for(lambda e, n=n: e.get("event") == "boot" and e.get("node") == n, timeout,
                      f"boot event for node {n}")


# ---------------------------------------------------------------------------------------------
def _read_dcf(path):
    cp = configparser.ConfigParser(interpolation=None, strict=False)
    cp.optionxform = str
    cp.read(path)
    return cp


def _compact(cp, index):
    """{sub: int} of a CompactSubObj object written as [xxxxValue]."""
    name = f"{index:04X}Value"
    if name not in cp:
        return {}
    return {int(k, 0): int(v, 0) for k, v in cp[name].items() if k != "NrOfEntries"}


def test_generated_dcf_writes_nothing_to_slaves(dcf_dir, eds):
    """dcfgen -r on bus.yml writes no concise DCF: no slave .bin, no 0x1F22 UploadFile, and -v
    lists no SDO. Made to fail on purpose with `heartbeat_producer: 200` on drive_3: dcfgen
    warned "no write access for sub-object 0x1017/0", wrote drive_3.bin, and Lely then
    downloaded 0x1017 = 200 (2B 17 10 00 C8 00) at boot and again after the slave restart."""
    cp = _read_dcf(dcf_dir / "master.dcf")
    log = (dcf_dir / "dcfgen.log").read_text()
    # No concise DCF at all: no slave .bin, no 0x1F22 entry, dcfgen -v lists no SDO write.
    assert sorted(p.name for p in dcf_dir.glob("*.bin")) == [], log
    assert not any(s.upper().startswith("1F22") and "UploadFile" in cp[s] for s in cp.sections())
    assert "writing" not in log, log
    # Heartbeat consumer 250 ms for every slave (0x1016: node << 16 | ms).
    assert sorted(_compact(cp, 0x1016).values()) == [n << 16 | 250 for n in NODES]
    # SYNC producer 10 ms (0x1006 in us; 0x1005 bit 30 = producer).
    assert int(cp["1006"]["DefaultValue"], 0) == 10000
    assert int(cp["1005"]["DefaultValue"], 0) & 0x40000000
    # 0x1F80 NMT startup: bit 0 master, bit 3 'do not start slaves' (dcfgen start_nodes: false).
    startup = int(cp["1F80"]["DefaultValue"], 0)
    assert startup & 0x01 and startup & 0x08, hex(startup)
    assert not startup & (0x02 | 0x10 | 0x40), hex(startup)
    # 0x1F81 per slave: bit 0 slave, bit 2 boot (identity uploads only), bit 3 mandatory,
    # bit 4 clear (reset communication allowed; setting it floods resets, see above).
    assert _compact(cp, 0x1F81) == {n: 0x0D for n in NODES}
    assert int(cp["1F89"]["DefaultValue"], 0) > 0  # finite boot time
    # Remote PDO objects: for each slave TPDO1-3 a 0x58xx (number << 8 | node) with its 0x5Axx
    # mapping, for each RPDO1-3 a 0x5Cxx with its 0x5Exx mapping, all equal to the EDS layouts
    # (read by wire.py, not by dcfgen).
    for kind, num_base, map_base in (("TPDO", 0x5800, 0x5A00), ("RPDO", 0x5C00, 0x5E00)):
        found = {}
        for i in range(0x200):
            name = f"{num_base + i:04X}"
            if name not in cp:
                break
            word = int(cp[name]["DefaultValue"], 0)
            found[(word >> 8, word & 0xFF)] = list(_compact(cp, map_base + i).values())
        expected = {}
        for n in NODES:
            for pdo in eds.pdos():
                if pdo["label"].startswith(kind):
                    words = [f["index"] << 16 | f["sub"] << 8 | f["bits"] for f in pdo["fields"]]
                    expected[(int(pdo["label"][4:]), n)] = words
        assert found == expected, kind


# ---------------------------------------------------------------------------------------------
def test_master_startup_traffic(start_probe, start_peer, capture, eds):
    """Observed with the calibrated bus.yml: the master's own boot-up 77F#00, then within
    microseconds one Reset Communication to all nodes, 000#8200 (the startup reset B15 allows;
    Lely 2.3.5 has no option that omits it, see reset_communication above), then SDO uploads of
    0x1000 and SYNC. No NMT Start; every slave heartbeat stays 0x00/0x7F; Lely reports boot
    state 127 (Pre-operational) with an empty status for each node. 'Before any other master
    frame' excludes the master's own boot-up, which Lely sends first."""
    cap = capture()
    start_probe()
    time.sleep(0.3)
    t0 = time.monotonic()
    peer = start_peer(duration_ms=3000)
    peer.wait_for(lambda e: e.get("event") == "exit", 6, "exit after 3 s")
    assert peer.wait_exit() == 0, peer.stderr
    frames = [f for f in cap.stop() if f[0] >= t0]

    # Preconditions: all six slaves are heard and Lely reports a boot result for each.
    for n in NODES:
        assert heartbeats(frames, n), f"INCONCLUSIVE: no heartbeat of node {n}"
    boots = {e["node"]: e for e in peer.of("boot")}
    print("boot events:", sorted((n, e["state"], e["es"], e["what"]) for n, e in boots.items()))
    assert sorted(boots) == list(NODES), f"INCONCLUSIVE: boot events {boots}"

    nmt = nmt_frames(frames)
    print("NMT frames:", [f"{t - t0:.3f}s {cs:02X} {node:02X}" for t, cs, node in nmt])
    master = [f for f in frames if f[1] not in slave_cob_ids(eds)]
    print("first master frames:", [fmt(f) for f in master[:4]])
    assert not [f for f in nmt if f[1] == NMT_START], "NMT Start on the wire"
    resets = [f for f in nmt if f[1] == NMT_RESET_COMM]
    assert len(resets) <= 1, resets
    others = [f for f in frames if f[1] not in slave_cob_ids(eds) | {wire.HEARTBEAT_BASE + MASTER_ID}
              and not (f[1] == wire.NMT_COB and f[2][0] == NMT_RESET_COMM)]
    if resets:
        assert not others or resets[0][0] <= others[0][0], (resets, fmt(others[0]))
        print("startup reset-communication:", f"{resets[0][1]:02X} {resets[0][2]:02X}")
    assert not [f for f in nmt if f[1] not in (NMT_RESET_COMM,)], nmt
    # Every slave stays Pre-operational after its (last) boot-up.
    for n in NODES:
        states = {s for _, s in heartbeats(frames, n)}
        assert states <= {BOOT_UP, PRE_OPERATIONAL}, (n, states)


# ---------------------------------------------------------------------------------------------
def test_master_does_not_reset_after_heartbeat_timeout(start_probe, start_peer, capture):
    """Default Lely 2.3.5: 0.25 s after SIGKILL of the slaves (consumer time 250 ms) Lely reports
    the timeout and sends NMT Reset Node 000#81nn to each timed-out slave (CiA 302 error
    handling in co_nmt_on_hb -> co_nmt_node_err_ind). start_nodes, mandatory and
    reset_communication did not change that. With the PeerMaster heartbeat indication override
    (main.cpp) the `hb` event still arrives 0.249 s after SIGKILL and no 0x000 frame follows."""
    cap = capture()
    probe = start_probe()
    peer = start_peer()
    boot_all(peer)
    # At least three heartbeats of every node after Lely booted it.
    t_boot = max(e["t"] for e in peer.of("boot"))
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        if all(len([h for h in heartbeats(cap.frames, n) if h[0] > t_boot]) >= 3 for n in NODES):
            break
        time.sleep(0.05)
    else:
        raise AssertionError("INCONCLUSIVE: fewer than 3 heartbeats per node after boot")
    t_kill = probe.kill()
    for n in NODES:
        peer.wait_for(lambda e, n=n: e.get("event") == "hb" and e.get("node") == n
                      and e.get("occurred") is True, 2, f"hb occurred for node {n}")
    t_hb = max(e["t"] for e in peer.of("hb", occurred=True))
    print(f"hb occurred {t_hb - t_kill:.3f} s after SIGKILL")
    time.sleep(max(0.0, t_hb + 3 - time.monotonic()))
    assert peer.terminate() == 0, peer.stderr
    frames = cap.stop()
    after = [fmt(f) for f in frames if f[1] == wire.NMT_COB and f[0] >= t_kill]
    assert after == [], f"NMT after heartbeat timeout: {after}"


# ---------------------------------------------------------------------------------------------
def test_master_sends_no_sdo_download_to_const_objects(start_probe, start_peer, capture):
    """Whole run with SIGKILL and restart of the slaves: Lely sends only expedited upload requests
    of 0x1000:00 (12 = six nodes x two boots), no download. After the restart each node's
    boot-up message makes Lely run the 'boot slave' process again (a second `boot` event), with
    no NMT frame of its own; the slaves stay 0x00/0x7F. With start_nodes: true the re-boot sent
    000#0101..0106 about 0.35 s after SIGKILL and the slaves went to 0x05: both asserts catch it."""
    cap = capture()
    probe = start_probe()
    peer = start_peer()
    boot_all(peer)
    t_kill = probe.kill()
    for n in NODES:
        peer.wait_for(lambda e, n=n: e.get("event") == "hb" and e.get("node") == n
                      and e.get("occurred") is True, 2, f"hb occurred for node {n}")
    t_restart = time.monotonic()
    start_probe()
    for n in NODES:
        peer.wait_for(lambda e, n=n: e.get("event") == "boot" and e.get("node") == n
                      and e["t"] > t_restart, 5, f"boot event for node {n} after the restart")
    time.sleep(0.5)
    assert peer.terminate() == 0, peer.stderr
    frames = cap.stop()

    print("NMT frames:", [f"{t - t_kill:+.3f}s {cs:02X} {node:02X}" for t, cs, node in
                          nmt_frames(frames)], "(relative to SIGKILL)")
    requests = sdo_requests(frames)
    before = [r for r in requests if r[0] < t_kill]
    after = [r for r in requests if r[0] > t_restart]
    assert before and after, f"INCONCLUSIVE: SDO traffic before/after restart {before} {after}"
    downloads = [r for r in requests if is_download_initiate(r[2])]
    print("SDO requests:", sorted(collections.Counter(
        (f"{r[3]:04X}:{r[4]:02X}", "download" if is_download_initiate(r[2]) else f"{r[2]:02X}")
        for r in requests).items()))
    assert [r for r in downloads if r[3] in CONST_OBJECTS] == [], downloads
    assert downloads == [], f"the master wrote objects: {downloads}"
    # The re-boot after the restart neither starts nor resets: no NMT frame from SIGKILL on, and
    # every restarted slave stays in boot-up/Pre-operational.
    after_kill = [fmt(f) for f in frames if f[1] == wire.NMT_COB and f[0] >= t_kill]
    assert after_kill == [], f"NMT after SIGKILL or during the re-boot: {after_kill}"
    for n in NODES:
        states = [s for t, s in heartbeats(frames, n) if t >= t_restart]
        assert states, f"INCONCLUSIVE: no heartbeat of node {n} after the restart"
        assert set(states) <= {BOOT_UP, PRE_OPERATIONAL}, (n, sorted(set(states)))


# ---------------------------------------------------------------------------------------------
def test_master_produces_sync(start_probe, start_peer, capture, eds):
    """SYNC 0x080 (no counter, 0x1019 = 0) every 10 ms: 150 frames in 1.5 s, interval p50
    10.000 ms, min/max within about +-0.15 ms on an idle host (printed, not asserted: the
    distribution is a measurement, not a CI threshold). Lely's OnSync fires for its own SYNC."""
    cap = capture()
    start_probe()
    peer = start_peer()
    time.sleep(0.5)
    t0 = time.monotonic()
    time.sleep(1.5)
    t1 = time.monotonic()
    assert peer.terminate() == 0, peer.stderr
    frames = cap.stop()
    sync = [t for t, cob, d in frames if cob == eds.sync_cob_id() and t0 <= t < t1]
    gaps = sorted((b - a) * 1e3 for a, b in zip(sync, sync[1:]))
    if gaps:
        q = statistics.quantiles(gaps, n=100) if len(gaps) > 1 else gaps
        print(f"SYNC frames in 1.5 s: {len(sync)}; interval ms min {gaps[0]:.3f} "
              f"p50 {statistics.median(gaps):.3f} p99 {q[-1]:.3f} max {gaps[-1]:.3f}")
    assert len(sync) >= 100, f"{len(sync)} SYNC frames in 1.5 s"
    assert [e for e in peer.of("sync") if t0 <= e["t"] < t1], "Lely reported no own SYNC"


# ---------------------------------------------------------------------------------------------
def test_master_does_not_reset_or_start_after_boot_failure(start_probe, start_peer, capture,
                                                             tmp_path):
    """Node 2 answers SDO but produces no heartbeat (0x1017 = 0 in its EDS copy); all six are
    mandatory. Lely ends node 2's 'boot slave' with es 'K', state 0, and sends nothing to it
    afterwards: no Start, no Reset, also after boot_time (2 s). The other five boot cleanly.
    This is the status character M2 phase A expects."""
    cap = capture()
    start_probe(nodes="1,3-6")
    start_probe(nodes="2", eds=derived_eds(tmp_path / "muted.eds", heartbeat_ms=0))
    t0 = time.monotonic()
    peer = start_peer()
    boot_all(peer, timeout=8)
    time.sleep(3)  # past boot_time (2 s) of the mandatory slaves
    assert peer.terminate() == 0, peer.stderr
    frames = cap.stop()
    boots = {e["node"]: e for e in peer.of("boot")}
    print("boot events:", sorted((n, e["state"], e["es"], e["what"]) for n, e in boots.items()))
    assert boots[2]["es"] != "", f"INCONCLUSIVE: node 2 booted without error: {boots[2]}"
    assert [n for n in NODES if n != 2 and boots[n]["es"] != ""] == [], boots
    muted = [s for t, s in heartbeats(frames, 2) if t >= t0 and s != BOOT_UP]
    assert muted == [], "INCONCLUSIVE: node 2 is not muted"
    nmt = [(t, cs, node) for t, cs, node in nmt_frames(frames) if t >= t0]
    print("NMT frames:", [f"{t - t0:.3f}s {cs:02X} {node:02X}" for t, cs, node in nmt])
    assert [f for f in nmt if f[1] != NMT_RESET_COMM] == [], nmt
    assert len(nmt) <= 1, nmt
