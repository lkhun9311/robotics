# SPDX-License-Identifier: Apache-2.0
"""Parsing of one captured observation, shared by the judges (judges.py, judges_master.py) and
the evidence scripts (tools/evidence/common.py), so a verdict and the image drawn from the same
recording read the frames the same way.

An observation's `host` and `device` hold [t, cob_id, hex data] per frame (conftest.Capture);
`bank` holds the vdrive_bank trace records. Nothing here judges or decides what is expected.
"""


def frames(obs, side, cobs=None, t0=None, t1=None, inclusive=False):
    """[(t s, COB-ID, bytes)] on `side` ('host' or 'device'), in capture order. `cobs` is one
    COB-ID or a set/tuple of them (None: every frame); t0/t1 bound the time, exclusive unless
    `inclusive`."""
    if isinstance(cobs, int):
        cobs = (cobs,)
    if inclusive:
        def within(t):
            return (t0 is None or t >= t0) and (t1 is None or t <= t1)
    else:
        def within(t):
            return (t0 is None or t > t0) and (t1 is None or t < t1)
    return [(t, c, bytes.fromhex(h)) for t, c, h in obs[side]
            if (cobs is None or c in cobs) and within(t)]


def pdo_fields(layout, data):
    """{field name: value} of one PDO frame by a wire.py layout (offset, bits, signed per field,
    little-endian). The caller decides what a frame of another length means."""
    return {f["name"]: int.from_bytes(data[f["offset"]:f["offset"] + f["bits"] // 8], "little",
                                      signed=f["signed"])
            for f in layout["fields"]}


def drive_records(obs, node, kind=None, t0=None, t1=None, bank="bank"):
    """The `drive` records of one node in a bank trace, optionally of one kind, with t_ns strictly
    between t0 and t1 (seconds)."""
    return [r for r in obs[bank] if r.get("event") == "drive" and r.get("node") == node and
            (kind is None or r.get("kind") == kind) and
            (t0 is None or r["t_ns"] / 1e9 > t0) and (t1 is None or r["t_ns"] / 1e9 < t1)]
