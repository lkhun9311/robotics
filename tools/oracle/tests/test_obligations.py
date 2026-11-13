# SPDX-License-Identifier: Apache-2.0
from oracle.obligations import check

MS = 1_000_000
DEADLINE = 4 * MS


def ev(t, kind, **fields):
    return {"t_ns": t, "clock": "c", "kind": kind, **fields}


def tr(t, frm, to, event):
    return ev(t, "state_transition", **{"from": frm, "to": to, "event": event})


def test_normal_run_where_everything_is_rejected_has_no_evidence():
    events = [ev(1, "command_sent", seq=1), ev(1, "command_rejected_drive", seq=1, reason="expired")]
    violations, missing = check("normal", events, DEADLINE)
    assert violations == [] and missing


def test_normal_stop_must_meet_the_deadline():
    events = [ev(1, "command_applied", seq=1), tr(10 * MS, "ENABLED", "STOPPING", "TRAJECTORY_DONE"),
              tr(20 * MS, "STOPPING", "READY", "STOP_CONFIRMED")]
    violations, missing = check("normal", events, DEADLINE)
    assert missing == [] and any("deadline" in v for v in violations)


def test_manager_stall_without_injection_or_block_has_no_evidence():
    _, missing = check("manager_stall", [ev(1, "command_applied", seq=1)], DEADLINE)
    assert any("manager_stall" in m for m in missing)
    assert any("PERMIT_EXPIRED" in m for m in missing)


def _stall(stop_t):
    return [ev(1, "permit_issued", manager=1, session=1, generation=1, expires_ns=39 * MS),
            ev(2, "command_applied", seq=1), ev(30 * MS, "fault_injected", name="manager_stall"),
            ev(39 * MS, "guard_block", seq=2, verdict="PERMIT_EXPIRED"), ev(stop_t, "drive_local_stop")]


def test_manager_stall_with_timely_stop():
    assert check("manager_stall", _stall(40 * MS), DEADLINE) == ([], [])


def test_manager_stall_late_stop_is_a_violation():
    violations, _ = check("manager_stall", _stall(50 * MS), DEADLINE)
    assert any("deadline" in v for v in violations)


def test_generation_switch_evidence():
    events = [ev(1, "permit_issued", manager=1, session=1, generation=1, expires_ns=48 * MS),
              ev(2, "permit_issued", manager=1, session=1, generation=2, expires_ns=60 * MS),
              ev(3, "guard_block", seq=5, verdict="GENERATION_MISMATCH"),
              ev(4, "command_applied", seq=6, session=1, generation=2, valid_until_ns=10 * MS)]
    assert check("generation_switch", events, DEADLINE) == ([], [])
    assert check("generation_switch", events[:3], DEADLINE)[1]


def _recovery(enable_before_reset=False):
    events = [ev(1, "enable_request"), ev(2, "command_applied", seq=1),
              ev(10 * MS, "fault_injected", name="injected_fault"),
              tr(12 * MS, "STOPPING", "FAULT_LATCHED", "STOP_CONFIRMED"),
              ev(20 * MS, "reset_request"), tr(20 * MS, "FAULT_LATCHED", "RECOVERING", "RESET_REQUEST")]
    if not enable_before_reset:
        events += [ev(30 * MS, "enable_request"), ev(31 * MS, "command_applied", seq=40),
                   ev(35 * MS, "command_redelivered", seq=1),
                   ev(35 * MS, "command_rejected_drive", seq=1, reason="stale_session")]
    return events


def test_fault_recovery_evidence():
    assert check("fault_reset_reenable", _recovery(), DEADLINE) == ([], [])


def test_initial_enable_does_not_count_as_recovery():
    _, missing = check("fault_reset_reenable", _recovery(enable_before_reset=True), DEADLINE)
    assert any("enable_request after the accepted reset" in m for m in missing)
