# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

REQUIRED = ("t_ns", "clock", "kind")


@dataclass
class LoadResult:
    status: str  # "ok" | "missing" | "empty" | "invalid"
    events: list = field(default_factory=list)
    error: str = ""


def load_trace(path) -> LoadResult:
    p = Path(path)
    if not p.exists():
        return LoadResult("missing", error=f"{p} does not exist")
    lines = [line for line in p.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not lines:
        return LoadResult("empty", error=f"{p} has no events")
    events: list[dict] = []
    last_t: int | None = None
    for number, line in enumerate(lines, start=1):
        try:
            event = json.loads(line)
        except json.JSONDecodeError as exc:
            return LoadResult("invalid", error=f"line {number}: {exc}")
        if not isinstance(event, dict):
            return LoadResult("invalid", error=f"line {number}: not an object")
        missing = [key for key in REQUIRED if key not in event]
        if missing:
            return LoadResult("invalid", error=f"line {number}: missing {missing}")
        if type(event["t_ns"]) is not int or not isinstance(event["kind"], str) \
                or not isinstance(event["clock"], str):
            return LoadResult("invalid", error=f"line {number}: t_ns must be int, kind and clock str")
        if last_t is not None and event["t_ns"] < last_t:
            return LoadResult("invalid", error=f"line {number}: time goes backwards")
        last_t = event["t_ns"]
        events.append(event)
    return LoadResult("ok", events)
