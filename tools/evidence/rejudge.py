#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Judges the committed recordings again, without vcan, and prints each verdict with its notes.

The notes are where the per-row numbers in README.md come from. Exit 1 if a recording is not
PASS.

  python3 tools/evidence/rejudge.py [--dir DIR] [m1 ... m6 f1 ... f7 r1]

--dir judges the observations of another directory instead, e.g. artifacts/interop of a CI
run's checks-artifacts (the vcan stage records every row it judges there).
"""
import argparse
import pathlib
import sys

from common import RECORDED, ROOT, load_recording, print_input, recording_command

sys.path.insert(0, str(ROOT / "tools" / "interop"))

import judges  # noqa: E402

ROWS = ["m1", "m2", "m3", "m4", "m5", "m6", "f1", "f7", "f2", "f3", "f4", "f5", "f6", "r1"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dir", default=str(RECORDED), help="directory of <row>.json.gz")
    parser.add_argument("rows", nargs="*", default=ROWS)
    args = parser.parse_args(argv)
    rc = 0
    for row in args.rows:
        shown = pathlib.Path(args.dir).resolve()
        shown = shown.relative_to(ROOT) if shown.is_relative_to(ROOT) else shown
        print_input(f"{shown}/{row}.json.gz", recording_command(row))
        status, text = judges.verdict(load_recording(row, args.dir))
        print(f"{row.upper()} {status}: {text}")
        rc |= status != "PASS"
    return rc


if __name__ == "__main__":
    sys.exit(main())
