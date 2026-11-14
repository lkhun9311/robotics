# SPDX-License-Identifier: Apache-2.0
import json

import functools

from oracle import rules
from oracle.rules import FAIL, INCONCLUSIVE, NOT_RUN, PASS, judge_file

# The `unit` scenario (no obligations) is reachable only from these tests, never from judge_file.
judge = functools.partial(rules.judge, allow_unit_scenario=True)

MS = 1_000_000
CLOCK = "fake_host_mono"


def ev(t, kind, clock=CLOCK, **fields):
    return {"t_ns": t, "clock": clock, "kind": kind, **fields}


def begin(scenario="unit"):
    return ev(0, "scenario_begin", scenario=scenario, defect="none", harness_version="1",
              lease_ns=48 * MS, stop_deadline_ns=4 * MS)


def end(t=40 * MS):
    return ev(t, "scenario_end")


def permit(t, session=1, generation=1, expires=48 * MS, manager=1):
    return ev(t, "permit_issued", manager=manager, session=session, generation=generation, expires_ns=expires)


def revoke(t, session=1, generation=1, manager=1):
    return ev(t, "permit_revoked", manager=manager, session=session, generation=generation)


def sent(t, seq=1, session=1, generation=1, valid_until=20 * MS):
    return ev(t, "command_sent", seq=seq, session=session, generation=generation, valid_until_ns=valid_until)


def applied(t, seq=1, session=1, generation=1, valid_until=20 * MS, clock=CLOCK):
    return ev(t, "command_applied", clock=clock, seq=seq, session=session, generation=generation,
              valid_until_ns=valid_until)


def rejected(t, seq=1, reason="expired"):
    return ev(t, "command_rejected_drive", seq=seq, reason=reason)


def transition(t, frm, to, event):
    return ev(t, "state_transition", **{"from": frm, "to": to, "event": event})


def base(applied_t=10 * MS, valid_until=20 * MS):
    return [begin(), permit(1 * MS), sent(applied_t, valid_until=valid_until),
            applied(applied_t, valid_until=valid_until), end()]


def reasons(v, text):
    return any(text in r for r in v.reasons)


def test_clean_trace_passes():
    assert judge(base()).result == PASS


def test_apply_exactly_at_valid_until_fails():
    v = judge(base(applied_t=20 * MS, valid_until=20 * MS))
    assert v.result == FAIL and reasons(v, "expired")


def test_apply_one_ns_before_valid_until_passes():
    assert judge(base(applied_t=20 * MS - 1, valid_until=20 * MS)).result == PASS


def test_apply_event_cannot_extend_the_sent_deadline():
    events = [begin(), permit(1 * MS), sent(4 * MS, valid_until=5 * MS),
              applied(10 * MS, valid_until=20 * MS), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "differs from sent") and reasons(v, "expired")


def test_command_outliving_permit_at_send_fails():
    events = [begin(), permit(1 * MS, expires=48 * MS), sent(10 * MS, valid_until=60 * MS),
              applied(10 * MS, valid_until=60 * MS), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "outlives")


def test_send_with_other_generation_fails_at_send_time():
    events = [begin(), permit(1 * MS), sent(10 * MS, generation=2), applied(10 * MS, generation=2), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "!= permit")


def test_send_before_any_permit_is_not_justified_later():
    events = [begin(), sent(1 * MS), permit(2 * MS), applied(3 * MS), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "no active permit")


def test_apply_after_revocation_fails():
    events = [begin(), permit(1 * MS), sent(4 * MS), revoke(5 * MS), applied(6 * MS), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "no active permit")


def test_outcome_for_unsent_command_fails():
    events = [begin(), permit(1 * MS), sent(10 * MS, seq=1), rejected(10 * MS, seq=1),
              applied(10 * MS, seq=2), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "never sent")


def test_duplicate_outcome_fails():
    events = base()
    events.insert(4, applied(10 * MS))
    v = judge(events)
    assert v.result == FAIL and reasons(v, "duplicate outcome")


def test_redelivered_command_may_only_be_rejected():
    ok = base()
    ok[4:4] = [ev(15 * MS, "command_redelivered", seq=1), rejected(15 * MS, reason="stale_session")]
    assert judge(ok).result == PASS
    bad = base()
    bad[4:4] = [ev(15 * MS, "command_redelivered", seq=1), applied(15 * MS)]
    v = judge(bad)
    assert v.result == FAIL and reasons(v, "redelivered command applied")


def test_foreign_clock_is_inconclusive_not_fail():
    # The apply would look expired, but times from another clock cannot prove anything.
    events = base()
    events[3] = applied(25 * MS, clock="sim_time")
    v = judge(events)
    assert v.result == INCONCLUSIVE and reasons(v, "clock")


def test_begin_must_be_first_and_end_last():
    events = base()
    events.append(ev(41 * MS, "drive_local_stop"))
    assert judge(events).result == INCONCLUSIVE
    events = base()
    events.insert(1, begin())
    assert judge(events).result == INCONCLUSIVE


def test_malformed_field_types_are_inconclusive_not_a_crash():
    events = base()
    events[1]["generation"] = []
    v = judge(events)
    assert v.result == INCONCLUSIVE and reasons(v, "must be int")


def _fault_then(extra, after=()):
    return [
        begin(),
        permit(1 * MS),
        ev(2 * MS, "fault_injected", name="injected_fault"),
        revoke(2 * MS),
        transition(3 * MS, "STOPPING", "FAULT_LATCHED", "STOP_CONFIRMED"),
        *extra,
        transition(9 * MS, "ARMING", "ENABLED", "ARMING_CONFIRMED"),
        permit(9 * MS, session=2, generation=5, expires=57 * MS),
        sent(10 * MS, session=2, generation=5),
        applied(10 * MS, session=2, generation=5),
        *after,
        end(),
    ]


ACCEPTED_RESET = [ev(4 * MS, "reset_request"), transition(4 * MS, "FAULT_LATCHED", "RECOVERING", "RESET_REQUEST"),
                  transition(4 * MS, "RECOVERING", "READY", "RECOVERY_VERIFIED")]
ACCEPTED_ENABLE = [ev(5 * MS, "enable_request"), transition(5 * MS, "READY", "ARMING", "ENABLE_REQUEST")]


def test_resume_after_fault_needs_accepted_reset_then_enable():
    assert judge(_fault_then([*ACCEPTED_RESET, *ACCEPTED_ENABLE])).result == PASS


def test_resume_without_enable_request_fails():
    v = judge(_fault_then(ACCEPTED_RESET))
    assert v.result == FAIL and reasons(v, "without explicit reset and enable")


def test_enable_request_before_reset_does_not_count():
    v = judge(_fault_then([ev(3 * MS, "enable_request"), *ACCEPTED_RESET,
                           transition(5 * MS, "READY", "ARMING", "ENABLE_REQUEST")]))
    assert v.result == FAIL


def test_rejected_reset_does_not_count():
    v = judge(_fault_then([ev(4 * MS, "reset_request"), *ACCEPTED_ENABLE]))
    assert v.result == FAIL and reasons(v, "without explicit reset and enable")


def test_recovery_with_the_pre_fault_identity_fails_even_without_a_revoke():
    events = [begin(), permit(1 * MS), ev(2 * MS, "fault_injected", name="injected_fault"),
              transition(3 * MS, "STOPPING", "FAULT_LATCHED", "STOP_CONFIRMED"), *ACCEPTED_RESET,
              ev(5 * MS, "enable_request"), transition(9 * MS, "ARMING", "ENABLED", "ARMING_CONFIRMED"),
              permit(9 * MS), sent(10 * MS), applied(10 * MS), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "reuses a closed session/generation")


def test_new_session_with_the_pre_fault_generation_fails():
    events = _fault_then([*ACCEPTED_RESET, *ACCEPTED_ENABLE])
    for e in events:
        if e["kind"] in ("permit_issued", "command_sent", "command_applied") and e["t_ns"] >= 9 * MS:
            e["generation"] = 1
    v = judge(events)
    assert v.result == FAIL and reasons(v, "reuses a closed session/generation")


def test_every_fault_must_latch_within_the_deadline():
    late_second = [ev(12 * MS, "fault_injected", name="injected_fault"),
                   transition(20 * MS, "STOPPING", "FAULT_LATCHED", "STOP_CONFIRMED")]
    v = judge(_fault_then([*ACCEPTED_RESET, *ACCEPTED_ENABLE], after=late_second))
    assert v.result == FAIL and reasons(v, "deadline")


def test_send_after_permit_expiry_fails():
    events = [begin(), permit(1 * MS, expires=5 * MS), sent(10 * MS, valid_until=5 * MS),
              rejected(10 * MS), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "sent after its permit expired")


def test_apply_after_a_shortened_permit_expires_fails():
    events = [begin(), permit(1 * MS, expires=48 * MS), sent(10 * MS, valid_until=20 * MS),
              permit(11 * MS, expires=12 * MS), applied(13 * MS, valid_until=20 * MS), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "applied after its permit expired")


def test_missing_latch_report_fails():
    events = [e for e in _fault_then([*ACCEPTED_RESET, *ACCEPTED_ENABLE])
              if e.get("to") != "FAULT_LATCHED"]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "never reported FAULT_LATCHED")


def test_second_fault_needs_its_own_latch_report():
    second = [ev(12 * MS, "fault_injected", name="injected_fault"), revoke(12 * MS, session=2, generation=5)]
    v = judge(_fault_then([*ACCEPTED_RESET, *ACCEPTED_ENABLE], after=second))
    assert v.result == FAIL and reasons(v, "never reported FAULT_LATCHED")


def test_apply_while_latched_fails():
    events = [begin(), permit(1 * MS), ev(2 * MS, "fault_injected", name="injected_fault"),
              sent(3 * MS), applied(3 * MS), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "while latched")


def test_reusing_a_closed_session_or_generation_fails():
    events = _fault_then([*ACCEPTED_RESET, *ACCEPTED_ENABLE])
    for e in events:
        if e["kind"] in ("permit_issued", "command_sent", "command_applied") and e["t_ns"] >= 9 * MS:
            e["session"], e["generation"] = 1, 1
    v = judge(events)
    assert v.result == FAIL and reasons(v, "reuses a closed session/generation")


def test_generation_going_backwards_fails():
    events = [begin(), permit(1 * MS, generation=1), permit(2 * MS, generation=2),
              permit(3 * MS, generation=1), sent(4 * MS), applied(4 * MS), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "backwards")


def test_stale_revoke_of_an_old_generation_keeps_the_current_permit():
    events = [begin(), permit(1 * MS, generation=1), permit(2 * MS, generation=2), revoke(3 * MS, generation=1),
              permit(4 * MS, generation=2), sent(5 * MS, generation=2), applied(5 * MS, generation=2), end()]
    assert judge(events).result == PASS


def test_sent_without_drive_outcome_is_inconclusive():
    events = [e for e in base() if e["kind"] != "command_applied"]
    v = judge(events)
    assert v.result == INCONCLUSIVE and reasons(v, "without drive outcome")


def test_no_commands_is_inconclusive():
    assert judge([e for e in base() if not e["kind"].startswith("command_")]).result == INCONCLUSIVE


def test_fail_wins_over_missing_evidence():
    events = [begin("normal"), permit(1 * MS), sent(10 * MS, generation=2), applied(10 * MS, generation=2), end()]
    assert judge(events).result == FAIL


def test_unknown_scenario_is_inconclusive():
    events = base()
    events[0] = begin("made_up")
    v = judge(events)
    assert v.result == INCONCLUSIVE and reasons(v, "no obligations")


def test_missing_file_is_not_run(tmp_path):
    assert judge_file(tmp_path / "none.jsonl").result == NOT_RUN


def test_empty_file_is_not_run(tmp_path):
    p = tmp_path / "t.jsonl"
    p.write_text("", encoding="utf-8")
    assert judge_file(p).result == NOT_RUN


def test_time_going_backwards_is_inconclusive(tmp_path):
    p = tmp_path / "t.jsonl"
    p.write_text(json.dumps(ev(5, "a")) + "\n" + json.dumps(ev(4, "b")) + "\n", encoding="utf-8")
    assert judge_file(p).result == INCONCLUSIVE


def test_failed_recovery_clears_the_accepted_reset():
    relatch = [ev(4 * MS, "reset_request"), transition(4 * MS, "FAULT_LATCHED", "RECOVERING", "RESET_REQUEST"),
               transition(4 * MS, "RECOVERING", "FAULT_LATCHED", "RECOVERY_FAILED"), *ACCEPTED_ENABLE]
    v = judge(_fault_then(relatch))
    assert v.result == FAIL and reasons(v, "without explicit reset and enable")


def test_rejected_enable_request_does_not_count():
    # The request arrives during RECOVERING and is rejected; production later enables by itself.
    events = _fault_then([ev(4 * MS, "reset_request"),
                          transition(4 * MS, "FAULT_LATCHED", "RECOVERING", "RESET_REQUEST"),
                          ev(4 * MS, "enable_request"),
                          transition(5 * MS, "RECOVERING", "READY", "RECOVERY_VERIFIED"),
                          transition(6 * MS, "READY", "ARMING", "ARMING_CONFIRMED")])
    v = judge(events)
    assert v.result == FAIL and reasons(v, "without explicit reset and enable")


def test_declared_deadline_must_match_the_oracle_contract():
    events = base()
    events[0]["stop_deadline_ns"] = 10**18
    v = judge(events)
    assert v.result == INCONCLUSIVE and reasons(v, "stop_deadline_ns")


def test_permit_longer_than_the_contract_lease_fails():
    events = [begin(), permit(1 * MS, expires=10**12), sent(10 * MS), applied(10 * MS), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "longer than the lease")


def test_renewing_an_expired_permit_fails():
    events = [begin(), permit(1 * MS, expires=5 * MS), permit(6 * MS, expires=50 * MS),
              sent(10 * MS), applied(10 * MS), end()]
    v = judge(events)
    assert v.result == FAIL and reasons(v, "renews an expired permit")


def test_unit_scenario_is_not_reachable_from_files(tmp_path):
    p = tmp_path / "t.jsonl"
    p.write_text("".join(json.dumps(e) + "\n" for e in base()), encoding="utf-8")
    v = judge_file(p)
    assert v.result == INCONCLUSIVE and reasons(v, "unit")


def test_unknown_fault_name_is_inconclusive():
    events = [*base()[:-1], ev(30 * MS, "fault_injected", name="not_a_known_fault"), end()]
    v = judge(events)
    assert v.result == INCONCLUSIVE and reasons(v, "unknown fault")

