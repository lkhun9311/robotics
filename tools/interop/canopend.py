# SPDX-License-Identifier: Apache-2.0
"""Start/stop helper for the pinned CANopenNode slave (`canopend`, node 2 by default), and the
small raw-CAN client the calibration tests talk to it with.

The binary is built into the dev image by docker/Dockerfile (CANopenLinux 87cfbac, CANopenNode
76b43c8; docker/canopennode_build.h: no self-start to Operational, log to stderr). Nothing
here imports our C++ master or tools/od: expectations come from cia301.py.

Storage. canopend keeps its persistent object-dictionary values in `<prefix>od_comm.persist`
(the part of the OD the 'save' command 0x1010:01 writes). The shipped OD has 0x1017 = 0 (no
heartbeat) and, without storage files, a first start raises a 0x5000 EMCY (non-volatile memory,
gate check 3). `seed_storage` produces a storage directory once, in test setup (not at image
build, which has no vcan): it starts canopend on a private vcan link `vcan_seed`, writes
0x1017 = 100 ms by SDO, stores the parameters with 0x1010:01 = 'save', and stops it. The
tests copy that directory per start, so a test's own 'save' or auto-save never leaks into the
next one. `-r` (reboot Linux on Reset Node) is never passed.
"""
import os
import pathlib
import shutil
import signal
import socket
import struct
import subprocess
import threading
import time

import cia301

BIN = pathlib.Path(os.environ.get("CANOPEND_BIN") or shutil.which("canopend") or "canopend")
NODE_ID = 2
SEED_IF = "vcan_seed"
HEARTBEAT_MS = 100  # the pre-seeded 0x1017

_CAN_FRAME = struct.Struct("=IB3x8s")
_SAVE = b"save"


class SdoAbort(Exception):
    def __init__(self, code):
        super().__init__(f"SDO abort 0x{code:08X}")
        self.code = code


class SdoTimeout(Exception):
    pass


class Bus:
    """A raw CAN socket on one interface: send frames, read them, expedited SDO client."""

    def __init__(self, ifname):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((ifname,))
        self.sock.settimeout(0.05)

    def close(self):
        self.sock.close()

    def send(self, cob_id, data=b""):
        self.sock.send(_CAN_FRAME.pack(cob_id, len(data), bytes(data).ljust(8, b"\0")))

    def nmt(self, command, node):
        self.send(cia301.NMT_COB, bytes([command, node]))

    def recv(self, timeout):
        """(cob_id, bytes) of the next frame, or None after timeout seconds."""
        deadline = time.monotonic() + timeout
        while True:
            try:
                raw = self.sock.recv(_CAN_FRAME.size)
            except socket.timeout:
                if time.monotonic() >= deadline:
                    return None
                continue
            cob, dlc, data = _CAN_FRAME.unpack(raw)
            return cob & 0x7FF, bytes(data[:dlc])

    def drain(self):
        self.sock.settimeout(0)
        try:
            while True:
                self.sock.recv(_CAN_FRAME.size)
        except (BlockingIOError, socket.timeout):
            pass
        finally:
            self.sock.settimeout(0.05)

    def _sdo(self, node, request, timeout):
        self.drain()
        self.send(cia301.SDO_RX_BASE + node, request)
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise SdoTimeout(f"no SDO response from node {node} within {timeout} s")
            frame = self.recv(left)
            if frame and frame[0] == cia301.SDO_TX_BASE + node and len(frame[1]) == 8:
                return frame[1]

    def sdo_upload(self, node, index, sub, timeout=1.0):
        """Expedited upload: the value as bytes. Raises SdoAbort(code) or SdoTimeout."""
        resp = self._sdo(node, struct.pack("<BHB4x", 0x40, index, sub), timeout)
        if resp[0] >> 5 == cia301.CS_ABORT:
            raise SdoAbort(struct.unpack_from("<I", resp, 4)[0])
        assert resp[0] >> 5 == cia301.SCS_UPLOAD_INITIATE and resp[0] & 2, resp.hex()
        size = 4 - ((resp[0] >> 2) & 3) if resp[0] & 1 else 4
        return resp[4:4 + size]

    def sdo_upload_int(self, node, index, sub, timeout=1.0):
        return int.from_bytes(self.sdo_upload(node, index, sub, timeout), "little")

    def sdo_download(self, node, index, sub, value, size, timeout=1.0):
        """Expedited download of an unsigned value of `size` bytes (1..4); bytes also accepted.
        Raises SdoAbort(code) or SdoTimeout."""
        data = value if isinstance(value, bytes) else int(value).to_bytes(size, "little")
        cmd = 0x20 | ((4 - len(data)) << 2) | 0x03
        resp = self._sdo(node, struct.pack("<BHB", cmd, index, sub) + data.ljust(4, b"\0"),
                         timeout)
        if resp[0] >> 5 == cia301.CS_ABORT:
            raise SdoAbort(struct.unpack_from("<I", resp, 4)[0])
        assert resp[0] >> 5 == cia301.SCS_DOWNLOAD_INITIATE, resp.hex()


class Canopend:
    """One canopend process; its stderr (the log, prefixed <priority>) is kept with the
    CLOCK_MONOTONIC time each line was read."""

    def __init__(self, ifname, storage_dir, node_id=NODE_ID, binary=None):
        self.ifname, self.node_id = ifname, node_id
        self.storage_dir = pathlib.Path(storage_dir)
        self.log = []  # (t, line)
        self._cond = threading.Condition()
        self.proc = subprocess.Popen(
            [str(binary or BIN), ifname, "-i", str(node_id), "-s", f"{self.storage_dir}/"],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        for line in self.proc.stderr:
            with self._cond:
                self.log.append((time.monotonic(), line.rstrip("\n")))
                self._cond.notify_all()

    def lines(self):
        with self._cond:
            return [line for _, line in self.log]

    def wait_log(self, needle, timeout=3.0):
        """The first log line containing needle, or None after timeout seconds."""
        deadline = time.monotonic() + timeout
        seen = 0
        with self._cond:
            while True:
                for _, line in self.log[seen:]:
                    if needle in line:
                        return line
                seen = len(self.log)
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self._cond.wait(left)

    def running(self):
        return self.proc.poll() is None

    def signal(self, sig):
        self.proc.send_signal(sig)

    def wait_exit(self, timeout=3.0):
        """Exit code of a canopend that is ending by itself, or None when it did not."""
        try:
            rc = self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None
        self._reader.join(timeout=2)
        return rc

    def stop(self):
        """SIGTERM (a clean exit); SIGKILL when it does not end within 3 s. Returns the code."""
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGCONT)
            self.proc.send_signal(signal.SIGTERM)
        rc = self.wait_exit(3.0)
        if rc is None:
            self.proc.kill()
            rc = self.proc.wait()
        self._reader.join(timeout=2)
        return rc


def ready(daemon, timeout=3.0):
    """canopend logs 'communication reset' then 'running ...' once its CAN socket is up."""
    return daemon.wait_log("running ...", timeout) is not None


def seed_storage(directory, binary=None):
    """Fills `directory` with the storage files of a canopend whose 0x1017 is HEARTBEAT_MS
    (see the module docstring). Needs NET_ADMIN to make the private link; raises RuntimeError
    with the cause when the seed cannot be made."""
    directory = pathlib.Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if subprocess.run(["ip", "link", "show", SEED_IF], capture_output=True).returncode != 0:
        made = subprocess.run(["ip", "link", "add", "dev", SEED_IF, "type", "vcan"],
                              capture_output=True, text=True)
        if made.returncode != 0:
            raise RuntimeError(f"cannot create {SEED_IF}: {made.stderr.strip()}")
    up = subprocess.run(["ip", "link", "set", "dev", SEED_IF, "up"], capture_output=True, text=True)
    if up.returncode != 0:
        raise RuntimeError(f"cannot raise {SEED_IF}: {up.stderr.strip()}")
    bus = Bus(SEED_IF)
    daemon = Canopend(SEED_IF, directory, binary=binary)
    try:
        if not ready(daemon):
            raise RuntimeError(f"canopend did not start: {daemon.lines()[-5:]}")
        bus.sdo_download(NODE_ID, 0x1017, 0, HEARTBEAT_MS, 2)
        bus.sdo_download(NODE_ID, 0x1010, 1, _SAVE, 4)
    finally:
        bus.close()
        daemon.stop()
    if not (directory / "od_comm.persist").exists():
        raise RuntimeError(f"no od_comm.persist after save in {directory}: {daemon.lines()[-5:]}")
    return directory
