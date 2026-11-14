#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Collect the facts E0 needs about the host or container: kernel, RT and vcan capability."""
from __future__ import annotations

import argparse
import gzip
import json
import platform
import resource
import subprocess
from pathlib import Path
from typing import Callable, Iterable

KERNEL_KEYS = (
    "CONFIG_PREEMPT_RT",
    "CONFIG_PREEMPT_DYNAMIC",
    "CONFIG_PREEMPT_LAZY",
    "CONFIG_RT_GROUP_SCHED",
    "CONFIG_HZ",
)

Runner = Callable[..., subprocess.CompletedProcess]


def parse_kernel_config(text: str, keys: Iterable[str]) -> dict[str, str | None]:
    values: dict[str, str] = {}
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("# CONFIG_") and line.endswith(" is not set"):
            values[line[2:-len(" is not set")]] = "n"
        elif line.startswith("CONFIG_") and "=" in line:
            key, value = line.split("=", 1)
            values[key] = value
    return {key: values.get(key) for key in keys}


def read_kernel_config(release: str) -> tuple[str | None, str]:
    boot = Path(f"/boot/config-{release}")
    if boot.exists():
        return boot.read_text(encoding="utf-8", errors="replace"), str(boot)
    proc = Path("/proc/config.gz")
    if proc.exists():
        return gzip.decompress(proc.read_bytes()).decode("utf-8", "replace"), str(proc)
    return None, "unavailable"


def _result(proc: subprocess.CompletedProcess) -> dict:
    return {"ok": proc.returncode == 0, "detail": (proc.stderr or proc.stdout or "").strip()}


def probe_sched_fifo(run: Runner = subprocess.run, priority: int = 50) -> dict:
    proc = run(["chrt", "-f", str(priority), "true"], capture_output=True, text=True)
    return _result(proc)


def probe_vcan(run: Runner = subprocess.run, name: str = "vcanprobe0") -> dict:
    add = run(["ip", "link", "add", "dev", name, "type", "vcan"], capture_output=True, text=True)
    if add.returncode != 0:
        return {**_result(add), "deleted": None}
    delete = run(["ip", "link", "del", "dev", name], capture_output=True, text=True)
    return {
        "ok": True,
        "deleted": delete.returncode == 0,
        "detail": f"created {name}; delete: {(delete.stderr or 'ok').strip()}",
    }


def probe_memlock() -> dict:
    soft, hard = resource.getrlimit(resource.RLIMIT_MEMLOCK)
    return {"soft": soft, "hard": hard}


def probe_mlock(size: int = 1 << 20, libc=None) -> dict:
    """Locks a real buffer. A generous RLIMIT_MEMLOCK alone does not prove mlock works."""
    import ctypes

    lib = libc if libc is not None else ctypes.CDLL(None, use_errno=True)
    buf = ctypes.create_string_buffer(size)
    addr = ctypes.addressof(buf)
    rc = lib.mlock(ctypes.c_void_p(addr), ctypes.c_size_t(size))
    if rc != 0:
        return {"ok": False, "detail": f"mlock({size}) failed errno={ctypes.get_errno()}"}
    lib.munlock(ctypes.c_void_p(addr), ctypes.c_size_t(size))
    return {"ok": True, "detail": f"mlock({size}) ok"}


VERSION_COMMANDS = {
    "gcc": ["g++", "--version"],
    "cmake": ["cmake", "--version"],
    "clang_tidy": ["clang-tidy", "--version"],
    "cyclictest": ["cyclictest", "--help"],
    "docker": ["docker", "--version"],
}


def probe_versions(run: Runner = subprocess.run) -> dict:
    versions = {}
    for name, cmd in VERSION_COMMANDS.items():
        try:
            proc = run(cmd, capture_output=True, text=True)
        except FileNotFoundError:
            versions[name] = "ERROR not installed"
            continue
        text = (proc.stdout or proc.stderr or "").strip().splitlines()
        first = text[0] if text else ""
        versions[name] = first if proc.returncode == 0 else f"ERROR rc={proc.returncode} {first}"
    return versions


PACKAGES = ("g++", "cmake", "clang-tidy", "clang-format", "libgtest-dev", "rt-tests", "gcovr",
            "python3-colcon-common-extensions")


def probe_packages(run: Runner = subprocess.run) -> dict:
    """Exact installed versions, so a rebuilt image can be compared with this one."""
    try:
        proc = run(["dpkg-query", "-W", "-f", "${Package}=${Version}\\n", *PACKAGES],
                   capture_output=True, text=True)
    except FileNotFoundError:
        return {"ERROR": "dpkg-query not available"}
    versions = {}
    for line in (proc.stdout or "").splitlines():
        name, _, version = line.partition("=")
        if name:
            versions[name] = version
    for name in PACKAGES:
        versions.setdefault(name, "ERROR not installed")
    return versions


def collect(run: Runner = subprocess.run) -> dict:
    release = platform.release()
    text, source = read_kernel_config(release)
    return {
        "kernel_release": release,
        "kernel_config_source": source,
        "kernel_config": parse_kernel_config(text, KERNEL_KEYS) if text else None,
        "sched_fifo_50": probe_sched_fifo(run),
        "vcan": probe_vcan(run),
        "memlock_limits": probe_memlock(),
        "mlock": probe_mlock(),
        "versions": probe_versions(run),
        "packages": probe_packages(run),
        "python": platform.python_version(),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", help="write JSON here instead of stdout")
    args = parser.parse_args(argv)
    text = json.dumps(collect(), indent=2, sort_keys=True)
    if args.out:
        Path(args.out).write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
