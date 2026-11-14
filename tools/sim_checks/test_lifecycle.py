# SPDX-License-Identifier: Apache-2.0
"""Start and stop of the simulator itself.

This module never requests the module-scoped `sim` fixture: that simulator stays up until its
own module ends, and a test here would meet it and trip the guard against a second simulator.
"""
import re

import pytest

from sim import Sim, SimError
from sim import ros2_control_pids as _ros2_control_pids


def _started(name):
    s = Sim(name)
    try:
        s.start()
    except SimError as e:
        reason = s.failure_report(e)
    else:
        return s
    s.stop()
    pytest.fail(reason)


def test_no_simulator_survives_teardown():
    s = _started("test_lifecycle-teardown")
    s.stop()
    assert _ros2_control_pids() == []


def test_start_refuses_when_a_simulator_is_already_running():
    a = _started("test_lifecycle-a")
    try:
        a_pids = _ros2_control_pids()
        assert a_pids, "simulator A has no ros2_control_node"
        b = Sim("test_lifecycle-b")
        with pytest.raises(SimError) as e:
            b.start()
        # Whole tokens: as a substring, pid 12 would match a message that names pid 123.
        named = set(re.findall(r"\b\d+\b", str(e.value)))
        assert set(a_pids) & named, f"refusal names {sorted(named)}, simulator A runs {a_pids}: {e.value}"
        b.stop()
    finally:
        a.stop()
    assert _ros2_control_pids() == []
