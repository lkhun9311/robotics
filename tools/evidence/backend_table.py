#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The E3c backend comparison table of README.md: one contract suite, two backends.

Runs the release builds of the contract suite against DirectBackend (direct_backend_tests,
Direct/Contract/*) and CanopenBackend over the in-memory drive bus (canopen_backend_tests,
Canopen/Contract/*), reads each case's result and the values it records (gtest XML properties,
hal_contract/testing/contract_suite.hpp), and prints one Markdown row per case. Both run in logic
time, so the values are the same on every run. --write-readme replaces the table between the two
marker comments in README.md; tools/evidence/tests/test_evidence.py checks that README.md holds
exactly what this prints.

  python3 tools/evidence/backend_table.py [--build build/release] [--write-readme]
"""
import argparse
import pathlib
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

from common import ROOT

README = ROOT / "README.md"
BEGIN = "<!-- backend table: python3 tools/evidence/backend_table.py --write-readme -->"
END = "<!-- end of backend table -->"
BACKENDS = [("Direct", "direct_backend/direct_backend_tests"),
            ("CANopen", "canopen_backend/canopen_backend_tests")]
PREFIX = {"Direct": "Direct/Contract/", "CANopen": "Canopen/Contract/"}
# The values each case records, in the order shown, with their unit. The reference of
# motion_end_ms and travel_rad is the last approval the device got, or the injected single-axis
# stop in InhibitStopsEveryAxisAndConfirms (contract_suite.hpp, expect_stopped_within_bound).
UNITS = {"arm_latency_ms": "arm {} ms", "fastest_rad_s": "fastest {} rad/s",
         "motion_end_ms": "motion ended {} ms after the reference",
         "travel_rad": "travel {} rad", "acceptance_latency_ms": "acceptance at most {} ms",
         "group_stop_latency_ms": "all axes stopped {} ms after the injection"}


def run(binary, prefix):
    """{case: (result, {key: value})} of one backend's contract cases."""
    if not binary.is_file():
        raise SystemExit(f"{binary} missing: build the workspace first (tools/ci/run_checks.sh)")
    with tempfile.TemporaryDirectory() as tmp:
        out = pathlib.Path(tmp) / "contract.xml"
        result = subprocess.run([str(binary), f"--gtest_filter={prefix}*",
                                 f"--gtest_output=xml:{out}"], capture_output=True, text=True)
        if not out.exists():
            raise SystemExit(f"{binary} wrote no report (exit {result.returncode}): "
                             f"{result.stdout[-1000:]}{result.stderr[-1000:]}")
        root = ET.parse(out).getroot()
    cases = {}
    for case in root.iter("testcase"):
        if not case.get("classname", "").startswith(prefix):
            continue
        failed = any(child.tag in ("failure", "error") for child in case)
        skipped = case.get("result") == "skipped" or any(c.tag == "skipped" for c in case)
        values = {p.get("name"): p.get("value") for p in case.iter("property")}
        cases[case.get("name")] = ("FAIL" if failed else "SKIPPED" if skipped else "PASS",
                                   values)
    if not cases:
        raise SystemExit(f"{binary}: no test matched {prefix}*")
    return cases


def cell(entry):
    if entry is None:
        return "MISSING"
    result, values = entry
    shown = [UNITS[k].format(values[k]) for k in UNITS if k in values]
    return result + (": " + "; ".join(shown) if shown else "")


def table(build):
    found = {name: run(build / path, PREFIX[name]) for name, path in BACKENDS}
    names = sorted(set().union(*found.values()))
    lines = ["| Contract case | DirectBackend | CanopenBackend (in-memory drive bus) |",
             "|---|---|---|"]
    for case in names:
        lines.append(f"| `{case}` | {cell(found['Direct'].get(case))} | "
                     f"{cell(found['CANopen'].get(case))} |")
    return "\n".join(lines) + "\n"


def readme_block(text):
    start = text.index(BEGIN) + len(BEGIN) + 1
    return text[start:text.index(END, start)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--build", default=str(ROOT / "build" / "release"))
    parser.add_argument("--write-readme", action="store_true")
    args = parser.parse_args(argv)
    build = pathlib.Path(args.build)
    print(f"input: {build / BACKENDS[0][1]} and {build / BACKENDS[1][1]} (the release build of "
          f"tools/ci/run_checks.sh)")
    rendered = table(build)
    if args.write_readme:
        text = README.read_text(encoding="utf-8")
        old = readme_block(text)
        README.write_text(text.replace(BEGIN + "\n" + old + END, BEGIN + "\n" + rendered + END),
                          encoding="utf-8")
        print(f"wrote the table into {README}")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
