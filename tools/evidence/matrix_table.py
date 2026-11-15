#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The interop matrix and the fault rows as a pass/fail table, from a vcan stage JUnit file.

The vcan stage of tools/ci/run_checks.sh writes artifacts/pytest-vcan.xml; CI uploads it in the
checks-artifacts bundle. Prints a Markdown table for README.md. A row whose test is missing from
the file is reported MISSING, never PASS. --stage e3b (the default) prints the Lely matrix and the
proxy fault rows (M1-M6, F1-F7, R1); --stage e3c prints our master's fault rows against our drives
(X1-X7) and our master against CANopenNode (N1-N7).

  python3 tools/evidence/matrix_table.py PATH/pytest-vcan.xml [--stage e3b|e3c] [--run-url URL]
"""
import argparse
import sys
import xml.etree.ElementTree as ET

from common import print_input

ROWS = [
    ("M1", "NMT states and service gating", "test_matrix", "test_nmt_states_and_service_gating"),
    ("M2", "boot-up and heartbeat", "test_matrix",
     "test_first_heartbeat_missing_vs_running_timeout"),
    ("M3", "SDO expedited transfers and aborts", "test_matrix", "test_sdo_expedited_and_aborts"),
    ("M4", "PDO bytes and DLC", "test_matrix", "test_pdo_bytes_and_lengths_seen_by_peer"),
    ("M5", "SYNC application and sampling order", "test_matrix",
     "test_sync_application_and_sampling_order"),
    ("M6", "EMCY production and reception", "test_matrix", "test_emcy_production_and_reception"),
    ("F1", "all RPDO2 delayed 60 ms", "test_proxy_faults",
     "test_all_late_rpdo2_times_out_to_quick_stop"),
    ("F7", "every fourth RPDO2 delayed 60 ms", "test_proxy_faults",
     "test_late_rpdo2_is_dropped_not_valid"),
    ("F2", "every third RPDO1 dropped", "test_proxy_faults",
     "test_drop_every_third_rpdo1_is_an_ordinary_gap"),
    ("F3", "RPDO1 and RPDO2 duplicated", "test_proxy_faults",
     "test_duplicate_fragments_change_nothing"),
    ("F4", "RPDO1 and RPDO2 reordered", "test_proxy_faults",
     "test_reorder_within_window_still_pairs"),
    ("F5", "conflicting RPDO1 copy", "test_proxy_faults",
     "test_conflicting_copy_is_an_integrity_stop"),
    ("F6", "TPDO1 dropped toward the master", "test_proxy_faults",
     "test_d2h_drop_hides_feedback_not_drive_state"),
    ("R1", "bank restart, old-tag replay", "test_proxy_faults",
     "test_restart_rejects_captured_old_tag_frames"),
]

E3C_ROWS = [
    ("X1", "node 3 TPDOs lost 35 ms", "test_master_faults",
     "test_short_feedback_outage_latches_inhibit"),
    ("X2", "node 3 TPDOs lost 100 ms", "test_master_faults",
     "test_dropped_tpdos_make_feedback_stale"),
    ("X3", "node 3 RPDO2 dropped", "test_master_faults",
     "test_one_node_command_loss_stops_the_group"),
    ("X4", "node 4 heartbeats dropped", "test_master_faults",
     "test_heartbeat_loss_raises_fault_without_reset"),
    ("X5", "node 5 SDO answer dropped during arm", "test_master_faults",
     "test_sdo_timeout_during_arm_fails_arming"),
    ("X6", "all RPDO2 delayed 60 ms", "test_master_faults",
     "test_all_commands_delayed_stop_the_group"),
    ("X7", "some node 3 RPDO2 delayed 60 ms", "test_master_faults",
     "test_minority_delayed_commands_never_actuate_late"),
    ("N1", "NMT states and service gating", "test_master_matrix",
     "test_master_nmt_states_and_service_gating"),
    ("N2", "boot-up and heartbeat", "test_master_matrix",
     "test_master_first_heartbeat_missing_vs_running_timeout"),
    ("N3", "SDO expedited transfers and aborts", "test_master_matrix",
     "test_master_sdo_expedited_and_aborts"),
    ("N4", "PDO bytes and DLC", "test_master_matrix", "test_master_pdo_bytes_and_lengths"),
    ("N5", "SYNC application and sampling order", "test_master_matrix",
     "test_master_sync_application_and_sampling_order"),
    ("N6", "EMCY reception", "test_master_matrix", "test_master_emcy_reception"),
    ("N7", "activation refused on missing capabilities", "test_master_matrix",
     "test_master_refuses_activation_on_canopennode"),
]
STAGES = {"e3b": ROWS, "e3c": E3C_ROWS}


def results(path):
    """{(module, test name): 'PASS' | 'FAIL' | 'SKIPPED'} from a pytest JUnit file."""
    root = ET.parse(path).getroot()
    out = {}
    for case in root.iter("testcase"):
        module = case.get("classname", "").rsplit(".", 1)[-1]
        tags = {child.tag for child in case}
        if tags & {"failure", "error"}:
            verdict = "FAIL"
        elif "skipped" in tags:
            verdict = "SKIPPED"
        else:
            verdict = "PASS"
        out[(module, case.get("name"))] = verdict
    return out


def table(path, stage="e3b"):
    found = results(path)
    lines = ["| Row | What | Test | Result |", "|---|---|---|---|"]
    for row, what, module, test in STAGES[stage]:
        verdict = found.get((module, test), "MISSING")
        lines.append(f"| {row} | {what} | `{module}.py::{test}` | {verdict} |")
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("junit")
    parser.add_argument("--stage", choices=sorted(STAGES), default="e3b")
    parser.add_argument("--run-url", default=None)
    args = parser.parse_args(argv)
    print_input(args.junit, "tools/ci/run_checks.sh (vcan stage) writes artifacts/pytest-vcan.xml; "
                "from CI: gh run download <run id> -n checks-artifacts")
    if args.run_url:
        print(f"run: {args.run_url}")
    print()
    print(table(args.junit, args.stage), end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
