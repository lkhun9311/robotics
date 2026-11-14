# SPDX-License-Identifier: Apache-2.0
"""Fixtures for the bus tests: vcan links, raw CAN capture, the Lely peer and calibration slaves.

These tests need vcan and NET_ADMIN. A missing link, binary or tool is a failure carrying
NOT_RUN or INCONCLUSIVE, never a skip.
"""
import json
import os
import pathlib
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(pathlib.Path(__file__).parent))
sys.path.insert(0, str(ROOT / "tools" / "od"))

import wire  # noqa: E402

BUS_YML = ROOT / "config" / "interop" / "bus.yml"
EDS = ROOT / "config" / "od" / "vdrive_arm.eds"
VCAN_UP = ROOT / "tools" / "bus" / "vcan_up.sh"
INSTALL = ROOT / "install" / "release" / "lely_peer" / "lib" / "lely_peer"
LELY_PEER_BIN = pathlib.Path(os.environ.get("LELY_PEER_BIN", INSTALL / "lely_peer"))
PROBE_SLAVE_BIN = pathlib.Path(os.environ.get("PROBE_SLAVE_BIN", INSTALL / "probe_slave"))
RELEASE = ROOT / "install" / "release"
VDRIVE_BANK_BIN = pathlib.Path(os.environ.get(
    "VDRIVE_BANK_BIN", RELEASE / "vdrive_bank" / "lib" / "vdrive_bank" / "vdrive_bank"))
BUS_FAULT_PROXY_BIN = pathlib.Path(os.environ.get(
    "BUS_FAULT_PROXY_BIN", RELEASE / "bus_fault_proxy" / "lib" / "bus_fault_proxy" / "bus_fault_proxy"))

HOST_IF = "vcan_host"
DEVICE_IF = "vcan_device"
READY_S = 2.0  # A process is ready when its first trace record is written within this.
NODES = (1, 2, 3, 4, 5, 6)

_CAN_FRAME = struct.Struct("=IB3x8s")
_CAN_EFF_FLAG, _CAN_RTR_FLAG, _CAN_ERR_FLAG = 0x80000000, 0x40000000, 0x20000000


def inconclusive(message):
    pytest.fail(f"INCONCLUSIVE: {message}", pytrace=False)


@pytest.fixture(scope="session")
def vcan():
    """Both vcan links up; exit 3 from vcan_up.sh is NOT_RUN and fails the test."""
    result = subprocess.run([str(VCAN_UP)], capture_output=True, text=True)
    if result.returncode == 3:
        pytest.fail(result.stdout.strip() or "NOT_RUN: vcan_up.sh exited 3", pytrace=False)
    assert result.returncode == 0, result.stdout + result.stderr
    return HOST_IF


@pytest.fixture(scope="session")
def dcf_dir(tmp_path_factory):
    """Master DCF generated from config/interop/bus.yml with remote PDO mappings (dcfgen -r)."""
    tool = shutil.which("dcfgen")
    assert tool is not None, "NOT_RUN: dcfgen not on PATH (ros-jazzy-lely-core-libraries)"
    out = tmp_path_factory.mktemp("dcf")
    result = subprocess.run([tool, "-r", "-v", "-d", str(out), str(BUS_YML)], cwd=ROOT,
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    (out / "dcfgen.log").write_text(result.stdout + result.stderr)
    return out


def derived_eds(path, *, heartbeat_ms=None):
    """A copy of our EDS for the Lely BasicSlave: 0x1F80 = 0x04 added, so the Lely slave stays in
    Pre-operational after boot-up like our drive (without 0x1F80 a Lely slave starts itself,
    gate check 2 'selfstart'); optionally a different 0x1017."""
    text = EDS.read_text()
    head, sep, rest = text.partition("[OptionalObjects]\nSupportedObjects=")
    count, _, rest = rest.partition("\n")
    n = int(count) + 1
    text = f"{head}{sep}{n}\n{n}=0x1F80\n{rest}"
    text += ("\n[1F80]\nParameterName=NMT startup\nObjectType=0x7\nDataType=0x0007\n"
             "AccessType=rw\nDefaultValue=0x00000004\nPDOMapping=0\n")
    if heartbeat_ms is not None:
        old = "[1017]\nParameterName=Producer heartbeat time\nObjectType=0x7\nDataType=0x0006\n" \
              "AccessType=const\nDefaultValue=100\n"
        assert old in text, "EDS 0x1017 block changed; update derived_eds"
        text = text.replace(old, old.replace("DefaultValue=100", f"DefaultValue={heartbeat_ms}"))
    path.write_text(text)
    return path


@pytest.fixture(scope="session")
def probe_eds(tmp_path_factory):
    return derived_eds(tmp_path_factory.mktemp("eds") / "probe.eds")


# SO_TIMESTAMPNS (asm-generic/socket.h; Python's socket module does not export it). On 64-bit
# Linux the control message carries a struct timespec of two 64-bit fields.
_SO_TIMESTAMPNS = 35
_TIMESPEC = struct.Struct("=qq")
CLOCK_OFFSET_SAMPLES = 50
CLOCK_DRIFT_LIMIT_NS = 1_000_000  # Realtime - monotonic may move this much during a capture.


def realtime_minus_monotonic_ns(samples=CLOCK_OFFSET_SAMPLES):
    """CLOCK_REALTIME - CLOCK_MONOTONIC in ns, from the tightest of `samples` monotonic/realtime/
    monotonic bracket reads (the one least disturbed by preemption)."""
    best = None
    for _ in range(samples):
        a = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        r = time.clock_gettime_ns(time.CLOCK_REALTIME)
        b = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        if best is None or b - a < best[0]:
            best = (b - a, r - (a + b) // 2)
    return best[1]


class Capture:
    """Every frame on one interface with its kernel receive time, as CLOCK_MONOTONIC seconds.

    The time is the socket's SO_TIMESTAMPNS stamp (set by the kernel when the frame was queued),
    not a time.monotonic() read after recv: the reader thread shares the GIL with the other
    capture and process-reader threads, so a Python-side read can lag the frame by a scheduler
    switch interval, while the judges compare these times with the bank's CLOCK_MONOTONIC trace
    at millisecond tolerances. The kernel stamps CLOCK_REALTIME; it is mapped to CLOCK_MONOTONIC
    by an offset sampled at capture start and checked again at stop (a clock step in between
    makes the capture INCONCLUSIVE). The global rule "kernel receive timestamps are diagnostics
    only" is about time passed to a Drive in vdrive_bank; this is the test's observation clock.
    """

    def __init__(self, ifname):
        self.ifname = ifname
        self.frames = []  # (t, cob_id, bytes)
        self.errors = []
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.setsockopt(socket.SOL_SOCKET, _SO_TIMESTAMPNS, 1)
        self._sock.bind((ifname,))
        self._sock.settimeout(0.05)
        self._offset_ns = realtime_minus_monotonic_ns()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stamp(self, ancdata):
        """CLOCK_MONOTONIC seconds of the kernel receive stamp in recvmsg's ancillary data."""
        for level, kind, data in ancdata:
            if level == socket.SOL_SOCKET and kind == _SO_TIMESTAMPNS and \
                    len(data) >= _TIMESPEC.size:
                sec, nsec = _TIMESPEC.unpack(data[:_TIMESPEC.size])
                return (sec * 1_000_000_000 + nsec - self._offset_ns) / 1e9
        return None

    def _run(self):
        while not self._stop.is_set():
            try:
                raw, ancdata, _, _ = self._sock.recvmsg(_CAN_FRAME.size,
                                                        socket.CMSG_SPACE(_TIMESPEC.size))
            except socket.timeout:
                continue
            except OSError as e:
                self.errors.append(str(e))
                return
            t = self.stamp(ancdata)
            if t is None:
                self.errors.append(f"a frame without a kernel receive timestamp: {ancdata}")
                return
            can_id, dlc, data = _CAN_FRAME.unpack(raw)
            if can_id & (_CAN_EFF_FLAG | _CAN_RTR_FLAG | _CAN_ERR_FLAG):
                self.errors.append(f"unexpected frame flags 0x{can_id:08x}")
                continue
            self.frames.append((t, can_id & 0x7FF, bytes(data[:dlc])))

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)
        self._sock.close()
        drift = realtime_minus_monotonic_ns() - self._offset_ns
        if abs(drift) > CLOCK_DRIFT_LIMIT_NS:
            self.errors.append(f"CLOCK_REALTIME moved {drift} ns against CLOCK_MONOTONIC during "
                               f"the capture; the kernel stamps cannot be mapped")
        assert not self.errors, f"INCONCLUSIVE: capture on {self.ifname}: {self.errors}"
        return self.frames


class Proc:
    """A child process whose stdout is JSON lines, read on a thread."""

    def __init__(self, name, argv, cwd=None, stdin=False):
        self.name = name
        self.argv = [str(a) for a in argv]
        self.events = []
        self.stderr = ""
        self.proc = subprocess.Popen(self.argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     stdin=subprocess.PIPE if stdin else None, text=True, cwd=cwd)
        self.lines_sent = 0
        self._cond = threading.Condition()
        self._out = threading.Thread(target=self._read, daemon=True)
        self._out.start()
        self._err = threading.Thread(target=self._read_err, daemon=True)
        self._err.start()

    def _read(self):
        for line in self.proc.stdout:
            try:
                event = json.loads(line)
            except ValueError:
                event = {"event": "unparsed", "line": line.rstrip("\n")}
            event.setdefault("t_rx", time.monotonic())
            with self._cond:
                self.events.append(event)
                self._cond.notify_all()

    def _read_err(self):
        self.stderr = self.proc.stderr.read()

    def wait_for(self, predicate, timeout, what):
        """First event satisfying predicate, or fail INCONCLUSIVE after timeout seconds."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                for event in self.events:
                    if predicate(event):
                        return event
                left = deadline - time.monotonic()
                if left <= 0 or self.proc.poll() is not None and not self._out.is_alive():
                    break
                self._cond.wait(min(left, 0.05))
        inconclusive(f"{self.name}: no {what} within {timeout} s (exit {self.proc.poll()}); "
                     f"stderr: {self.stderr[-500:]!r}; events: {self.events[-10:]}")

    def send(self, line):
        """One console line (stdin); returns its number n, as in the peer's cmd/cmd_done."""
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()
        self.lines_sent += 1
        return self.lines_sent

    def command(self, line, timeout=5):
        """Sends a console line and waits until the peer reports it done; returns cmd_done."""
        n = self.send(line)
        done = self.wait_for(lambda e: e.get("event") == "cmd_done" and e.get("n") == n, timeout,
                             f"cmd_done for {line!r}")
        if not done["ok"]:
            inconclusive(f"{self.name}: console line {line!r} was not carried out")
        return done

    def of(self, kind, **fields):
        with self._cond:
            return [e for e in self.events
                    if e.get("event") == kind and all(e.get(k) == v for k, v in fields.items())]

    def terminate(self, timeout=5):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
        try:
            rc = self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
            inconclusive(f"{self.name} did not exit within {timeout} s of SIGTERM")
        self._out.join(timeout=2)
        self._err.join(timeout=2)
        return rc

    def wait_exit(self, timeout=2):
        """Exit status of a process that is ending by itself."""
        try:
            rc = self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            inconclusive(f"{self.name} did not exit within {timeout} s")
        self._out.join(timeout=2)
        self._err.join(timeout=2)
        return rc

    def kill(self):
        """SIGKILL; returns the CLOCK_MONOTONIC time the signal was sent."""
        t = time.monotonic()
        self.proc.kill()
        self.proc.wait()
        self._out.join(timeout=2)
        return t


@pytest.fixture
def procs():
    started = []
    yield started
    for p in started:
        if p.proc.poll() is None:
            p.proc.kill()
            p.proc.wait()


@pytest.fixture
def start_probe(vcan, procs, probe_eds):
    def start(nodes="1-6", eds=None):
        assert PROBE_SLAVE_BIN.exists(), f"NOT_RUN: {PROBE_SLAVE_BIN} not built"
        p = Proc("probe_slave", [PROBE_SLAVE_BIN, "--if", vcan, "--eds", eds or probe_eds,
                                 "--nodes", nodes])
        procs.append(p)
        p.wait_for(lambda e: e.get("event") == "ready", 2, "ready line")
        return p
    return start


@pytest.fixture
def start_peer(vcan, procs, dcf_dir):
    def start(scenario="idle", duration_ms=None, node=None, count=None, statusword=None,
              validity_ms=None):
        """console: stdin is a pipe (Proc.send, Proc.command). statusword="sdo": the scenario
        reads 0x6041 by SDO instead of waiting for TPDO1. validity_ms: pdo_stream's
        valid_until = now + validity_ms (the peer's default is 40)."""
        assert LELY_PEER_BIN.exists(), f"NOT_RUN: {LELY_PEER_BIN} not built"
        argv = [LELY_PEER_BIN, "--if", vcan, "--dcf", dcf_dir / "master.dcf",
                "--scenario", scenario]
        if duration_ms is not None:
            argv += ["--duration-ms", duration_ms]
        if node is not None:
            argv += ["--node", node]
        if count is not None:
            argv += ["--count", count]
        if statusword is not None:
            argv += ["--statusword", statusword]
        if validity_ms is not None:
            argv += ["--validity-ms", validity_ms]
        # cwd = the DCF directory: a concise DCF (UploadFile=<name>.bin, relative) would be found
        # and downloaded, so a configuration write cannot hide behind a missing file.
        p = Proc("lely_peer", argv, cwd=dcf_dir, stdin=scenario == "console")
        procs.append(p)
        p.wait_for(lambda e: e.get("event") == "start", 2, "start line")
        return p
    return start


@pytest.fixture
def capture(vcan):
    caps = []

    def start(ifname=HOST_IF):
        c = Capture(ifname)
        caps.append(c)
        return c
    yield start
    for c in caps:
        if not c._stop.is_set():
            c._stop.set()
            c._thread.join(timeout=2)
            c._sock.close()


@pytest.fixture(scope="session")
def eds():
    return wire.Eds(EDS)


class TraceFile:
    """The JSON lines a process writes to its --trace file, read as they arrive."""

    def __init__(self, path):
        self.path = pathlib.Path(path)
        self.records = []
        self._offset = 0
        self._partial = ""

    def poll(self):
        if not self.path.exists():
            return self.records
        with open(self.path, encoding="utf-8") as f:
            f.seek(self._offset)
            text = f.read()
            self._offset = f.tell()
        lines = (self._partial + text).split("\n")
        self._partial = lines.pop()
        for line in lines:
            try:
                self.records.append(json.loads(line))
            except ValueError:
                inconclusive(f"{self.path}: unparsable trace line {line!r}")
        return self.records

    def of(self, event):
        return [r for r in self.poll() if r.get("event") == event]

    def wait_for(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while True:
            for r in self.poll():
                if predicate(r):
                    return r
            if time.monotonic() >= deadline:
                inconclusive(f"{self.path.name}: no {what} within {timeout} s; "
                             f"last records: {self.records[-5:]}")
            time.sleep(0.01)


class TracedProc:
    """A process that writes its trace to a file (vdrive_bank, bus_fault_proxy).

    Ready: its first trace record within READY_S. Stopped (SIGTERM): exit 0 and a final `exit`
    record with trace_overflow == 0, so a test never judges an incomplete trace."""

    def __init__(self, name, argv, trace_path):
        self.name = name
        self.trace = TraceFile(trace_path)
        self.stderr = ""
        self.exit = None
        self.proc = subprocess.Popen([str(a) for a in argv], stdout=subprocess.DEVNULL,
                                     stderr=subprocess.PIPE, text=True)
        self._err = threading.Thread(target=self._read_err, daemon=True)
        self._err.start()

    def _read_err(self):
        self.stderr = self.proc.stderr.read()

    def ready(self):
        deadline = time.monotonic() + READY_S
        while not self.trace.poll():
            if self.proc.poll() is not None:
                self._err.join(timeout=2)
                inconclusive(f"{self.name} exited {self.proc.returncode} before its first "
                             f"trace record; stderr: {self.stderr[-500:]!r}")
            if time.monotonic() >= deadline:
                inconclusive(f"{self.name}: no trace record within {READY_S} s")
            time.sleep(0.01)
        return self

    @property
    def harness_path(self):
        """The harness's own records about this process (B7 `killed`), next to its trace."""
        return self.trace.path.with_suffix(".harness.jsonl")

    def kill(self):
        """SIGKILL (B7): no exit record is expected from the process. The harness writes a
        `killed` record (CLOCK_MONOTONIC time.monotonic(), signal, exit status); the trace is judged
        on readiness and the records up to the kill. Returns the killed record."""
        if self.proc.poll() is not None:
            inconclusive(f"{self.name} had already exited ({self.proc.returncode}) before the kill")
        t = time.monotonic()
        self.proc.send_signal(signal.SIGKILL)
        rc = self.proc.wait(timeout=5)
        self._err.join(timeout=2)
        self.trace.poll()
        record = {"event": "killed", "t_mono": t, "signal": int(signal.SIGKILL),
                  "exit_status": rc}
        with open(self.harness_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record) + "\n")
        self.exit = record
        return record

    def wait_exit(self, timeout):
        """Exit status of a process that ends by itself (an error exit); its trace is not judged
        here and teardown does not stop it again. Fails INCONCLUSIVE if it is still running."""
        try:
            rc = self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            inconclusive(f"{self.name} still running {timeout} s later")
        self._err.join(timeout=2)
        self.trace.poll()
        self.exit = {"event": "ended", "exit_status": rc}
        return rc

    def stop(self, timeout=5):
        """SIGTERM, then the completeness contract (B7). Returns the exit record."""
        if self.exit is not None:
            return self.exit
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
        try:
            rc = self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
            inconclusive(f"{self.name} did not exit within {timeout} s of SIGTERM")
        self._err.join(timeout=2)
        assert rc == 0, f"{self.name} exited {rc}; stderr: {self.stderr[-500:]!r}"
        records = self.trace.poll()
        if not records or records[-1].get("event") != "exit":
            inconclusive(f"{self.name}: trace has no final exit record: {records[-3:]}")
        self.exit = records[-1]
        if self.exit.get("trace_overflow") != 0:
            inconclusive(f"{self.name}: trace incomplete, trace_overflow = "
                         f"{self.exit.get('trace_overflow')}")
        return self.exit


@pytest.fixture
def traced(tmp_path):
    started = []
    yield started
    errors = []
    for p in started:
        try:
            p.stop()
        except BaseException as e:  # noqa: BLE001 - every process is stopped, then reported.
            if p.proc.poll() is None:
                p.proc.kill()
                p.proc.wait()
            errors.append(f"{p.name}: {e}")
    if errors:
        pytest.fail("; ".join(errors), pytrace=False)


@pytest.fixture
def start_bank(vcan, traced, tmp_path):
    """vdrive_bank on vcan_device, ready; stopped cleanly at teardown."""
    def start(inject=False, hb_muted=None):
        assert VDRIVE_BANK_BIN.exists(), f"NOT_RUN: {VDRIVE_BANK_BIN} not built"
        n = len(traced)
        trace = tmp_path / f"bank{n}.jsonl"
        argv = [VDRIVE_BANK_BIN, "--if", DEVICE_IF, "--trace", trace]
        if hb_muted is not None:
            argv += ["--hb-muted", hb_muted]
        inject_path = tmp_path / f"bank{n}.sock" if inject else None
        if inject:
            argv += ["--inject", inject_path]
        p = TracedProc("vdrive_bank", argv, trace)
        p.inject_path = inject_path
        traced.append(p)
        return p.ready()
    return start


@pytest.fixture
def bank(start_bank):
    return start_bank()


@pytest.fixture
def start_proxy(vcan, traced, tmp_path):
    """bus_fault_proxy between vcan_host and vcan_device, ready; stopped cleanly at teardown."""
    def start(rules=None):
        assert BUS_FAULT_PROXY_BIN.exists(), f"NOT_RUN: {BUS_FAULT_PROXY_BIN} not built"
        n = len(traced)
        trace = tmp_path / f"proxy{n}.jsonl"
        argv = [BUS_FAULT_PROXY_BIN, "--host", HOST_IF, "--device", DEVICE_IF, "--trace", trace]
        if rules is not None:
            path = tmp_path / f"proxy{n}.rules"
            path.write_text(rules)
            argv += ["--rules", path]
        p = TracedProc("bus_fault_proxy", argv, trace)
        traced.append(p)
        return p.ready()
    return start


@pytest.fixture
def proxy(start_proxy):
    return start_proxy()


class RawPeer:
    """Sends raw classic frames on one interface; returns the CLOCK_MONOTONIC send time."""

    def __init__(self, ifname):
        self.ifname = ifname
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.bind((ifname,))

    def send(self, cob_id, data=b"", flags=0):
        raw = _CAN_FRAME.pack(cob_id | flags, len(data), bytes(data).ljust(8, b"\0"))
        t = time.monotonic()
        self._sock.send(raw)
        return t

    def close(self):
        self._sock.close()


@pytest.fixture
def peer(vcan):
    peers = []

    def open_peer(ifname):
        p = RawPeer(ifname)
        peers.append(p)
        return p
    yield open_peer
    for p in peers:
        p.close()


class InjectClient:
    """A B6 client: one bound UNIX datagram socket, so the bank can answer."""

    def __init__(self, server, path):
        self.server = str(server)
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._sock.bind(str(path))
        self._sock.settimeout(1.0)

    def ask(self, command):
        self._sock.sendto(command, self.server)
        try:
            return self._sock.recv(256)
        except socket.timeout:
            inconclusive(f"no reply to {command[:40]!r} within 1 s")

    def close(self):
        self._sock.close()


@pytest.fixture
def inject_client(tmp_path):
    clients = []

    def open_client(bank):
        assert bank.inject_path is not None, "the bank was started without --inject"
        c = InjectClient(bank.inject_path, tmp_path / f"client{len(clients)}.sock")
        clients.append(c)
        return c
    yield open_client
    for c in clients:
        c.close()
