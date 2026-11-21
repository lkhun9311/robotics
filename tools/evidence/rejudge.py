#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Judges the committed recordings again, without vcan, and prints each verdict with its notes.

The notes are where the per-row numbers in README.md come from. Exit 1 if a recording is not
PASS.

  python3 tools/evidence/rejudge.py [--dir DIR] [m1 ... m6 f1 ... f7 r1 x1 ... x7 n1 ... n7 x7_edge]

The committed n1..n7 (our master against CANopenNode) are in tools/interop/recorded/master; a
--dir directory holds every row side by side (the vcan stage writes them so).
The committed set also holds the kept regression recordings (common.KEPT, e.g. x7_edge), judged by
their row's judge; a --dir directory is not expected to have them.

--dir judges the observations of another directory instead, e.g. artifacts/interop of a CI
run's checks-artifacts (the vcan stage records every row it judges there).
"""
import argparse
import pathlib
import sys

from common import (KEPT, RECORDED, ROOT, load_recording, print_input, recorded_dir,
                    recording_command)

sys.path.insert(0, str(ROOT / "tools" / "interop"))

import judges  # noqa: E402
import judges_master  # noqa: E402

# How a directory other than the committed recordings is written (CI: artifacts/interop).
VCAN_STAGE_COMMAND = ("the vcan stage of tools/ci/run_checks.sh (INTEROP_RECORD_DIR="
                      "artifacts/interop); from CI: gh run download <run id> -n checks-artifacts")
ROWS = ["m1", "m2", "m3", "m4", "m5", "m6", "f1", "f7", "f2", "f3", "f4", "f5", "f6", "r1",
        "x1", "x2", "x3", "x4", "x5", "x6", "x7", "n1", "n2", "n3", "n4", "n5", "n6", "n7"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dir", default=str(RECORDED), help="directory of <row>.json.gz")
    parser.add_argument("rows", nargs="*")
    args = parser.parse_args(argv)
    directory = pathlib.Path(args.dir).resolve()
    committed = directory == RECORDED.resolve()
    rows = args.rows or (ROWS + list(KEPT) if committed else ROWS)
    rc = 0
    for row in rows:
        command = recording_command(row) if committed else VCAN_STAGE_COMMAND
        where = recorded_dir(row).resolve() if committed else directory
        where_shown = where.relative_to(ROOT) if where.is_relative_to(ROOT) else where
        print_input(f"{where_shown}/{row}.json.gz", command)
        if not (where / f"{row}.json.gz").exists():
            print(f"{row.upper()} NOT RECORDED: no observation in {where_shown} (its test failed "
                  f"before judge(), or the row did not run)")
            rc = 1
            continue
        module = judges_master if row.startswith("x") else judges
        status, text = module.verdict(load_recording(row, where))
        print(f"{row.upper()} {status}: {text}")
        rc |= status != "PASS"
    return rc


if __name__ == "__main__":
    sys.exit(main())
