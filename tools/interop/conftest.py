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

HOST_IF = "vcan_host"
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


class Capture:
    """Every frame on one interface with its CLOCK_MONOTONIC receive time (time.monotonic())."""

    def __init__(self, ifname):
        self.ifname = ifname
        self.frames = []  # (t, cob_id, bytes)
        self.errors = []
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.bind((ifname,))
        self._sock.settimeout(0.05)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                raw = self._sock.recv(_CAN_FRAME.size)
            except socket.timeout:
                continue
            except OSError as e:
                self.errors.append(str(e))
                return
            t = time.monotonic()
            can_id, dlc, data = _CAN_FRAME.unpack(raw)
            if can_id & (_CAN_EFF_FLAG | _CAN_RTR_FLAG | _CAN_ERR_FLAG):
                self.errors.append(f"unexpected frame flags 0x{can_id:08x}")
                continue
            self.frames.append((t, can_id & 0x7FF, bytes(data[:dlc])))

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)
        self._sock.close()
        assert not self.errors, f"INCONCLUSIVE: capture on {self.ifname}: {self.errors}"
        return self.frames


class Proc:
    """A child process whose stdout is JSON lines, read on a thread."""

    def __init__(self, name, argv, cwd=None):
        self.name = name
        self.argv = [str(a) for a in argv]
        self.events = []
        self.stderr = ""
        self.proc = subprocess.Popen(self.argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     text=True, cwd=cwd)
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
    def start(scenario="idle", duration_ms=None):
        assert LELY_PEER_BIN.exists(), f"NOT_RUN: {LELY_PEER_BIN} not built"
        argv = [LELY_PEER_BIN, "--if", vcan, "--dcf", dcf_dir / "master.dcf",
                "--scenario", scenario]
        if duration_ms is not None:
            argv += ["--duration-ms", duration_ms]
        # cwd = the DCF directory: a concise DCF (UploadFile=<name>.bin, relative) would be found
        # and downloaded, so a configuration write cannot hide behind a missing file.
        p = Proc("lely_peer", argv, cwd=dcf_dir)
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
