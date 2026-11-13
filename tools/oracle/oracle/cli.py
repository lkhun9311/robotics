# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import argparse
import json
import sys

from oracle.rules import EXIT_CODES, judge_file


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Judge a runtime trace independently of production code.")
    parser.add_argument("trace")
    args = parser.parse_args(argv)
    verdict = judge_file(args.trace)
    print(json.dumps({"result": verdict.result, "reasons": verdict.reasons}))
    return EXIT_CODES[verdict.result]


if __name__ == "__main__":
    sys.exit(main())
