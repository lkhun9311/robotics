# SPDX-License-Identifier: Apache-2.0
import subprocess

import env_manifest

CONFIG = """
CONFIG_PREEMPT_DYNAMIC=y
# CONFIG_PREEMPT_RT is not set
CONFIG_HZ=1000
"""


def test_parse_kernel_config_reads_set_unset_and_missing():
    got = env_manifest.parse_kernel_config(
        CONFIG, ["CONFIG_PREEMPT_DYNAMIC", "CONFIG_PREEMPT_RT", "CONFIG_HZ", "CONFIG_RT_GROUP_SCHED"]
    )
    assert got == {
        "CONFIG_PREEMPT_DYNAMIC": "y",
        "CONFIG_PREEMPT_RT": "n",
        "CONFIG_HZ": "1000",
        "CONFIG_RT_GROUP_SCHED": None,
    }


def _runner(returncode, stderr=""):
    def run(cmd, **_kwargs):
        return subprocess.CompletedProcess(cmd, returncode, stdout="", stderr=stderr)

    return run


def test_sched_fifo_probe_reports_success_and_failure():
    assert env_manifest.probe_sched_fifo(run=_runner(0))["ok"] is True
    failed = env_manifest.probe_sched_fifo(run=_runner(1, "Operation not permitted"))
    assert failed["ok"] is False
    assert "Operation not permitted" in failed["detail"]


def test_vcan_probe_fails_when_link_add_fails():
    result = env_manifest.probe_vcan(run=_runner(2, "RTNETLINK answers: Operation not supported"))
    assert result["ok"] is False
    assert "not supported" in result["detail"]


def test_vcan_probe_reports_failed_delete():
    calls = []

    def run(cmd, **_kwargs):
        calls.append(cmd)
        code = 0 if "add" in cmd else 1
        return subprocess.CompletedProcess(cmd, code, stdout="", stderr="busy" if code else "")

    result = env_manifest.probe_vcan(run=run)
    assert result["ok"] is True
    assert result["deleted"] is False


def test_memlock_probe_reports_limits():
    result = env_manifest.probe_memlock()
    assert set(result) == {"soft", "hard"}


class _FakeLibc:
    def __init__(self, rc):
        self.rc = rc

    def mlock(self, _addr, _size):
        return self.rc

    def munlock(self, _addr, _size):
        return 0


def test_mlock_probe_calls_mlock():
    assert env_manifest.probe_mlock(libc=_FakeLibc(0))["ok"] is True
    assert env_manifest.probe_mlock(libc=_FakeLibc(-1))["ok"] is False


def test_versions_probe_records_failures():
    versions = env_manifest.probe_versions(run=_runner(127, "not found"))
    assert all(v.startswith("ERROR") for v in versions.values())


def test_packages_probe_marks_missing_packages():
    def run(cmd, **_kwargs):
        return subprocess.CompletedProcess(cmd, 1, stdout="cmake=3.28.3-1build7\n", stderr="")

    packages = env_manifest.probe_packages(run=run)
    assert packages["cmake"] == "3.28.3-1build7"
    assert packages["rt-tests"].startswith("ERROR")
