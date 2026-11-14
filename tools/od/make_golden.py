# SPDX-License-Identifier: Apache-2.0
"""Regenerate the golden CAN frames under src/canopen_codec/test/golden/ from the EDS.

The JSON files cannot carry a license header; they are generated output covered by the
repository license. Output is deterministic: a test fails if the committed files differ from
a fresh generation. Usage: python3 tools/od/make_golden.py [output_dir]
"""
import json
import pathlib
import sys

import wire

GOLDEN_DIR = pathlib.Path(__file__).resolve().parents[2] / "src" / "canopen_codec" / "test" / "golden"
NODES = (1, 6)
ABORTS = ((0x06020000, "object does not exist"), (0x06010002, "write to read-only object"),
          (0x06090030, "value out of range"),
          (0x05040001, "command specifier not valid or unknown (segmented and block requests)"))


def families(eds):
    nmt = [wire.nmt(c, n) for c in wire.NMT_COMMANDS for n in NODES]
    nmt.append(wire.nmt("start", 127))
    nmt.append(wire.nmt("reset_node", 0))  # node 0 addresses every node

    heartbeat = [wire.heartbeat(s, n) for s in wire.HEARTBEAT_STATES for n in NODES]
    heartbeat.append(wire.heartbeat("operational", 127))

    syncs = [wire.sync(eds)]

    emcy = [wire.emcy(eds, n, code, reg, mfr) for n in NODES
            for code, reg, mfr in ((0x0000, 0x00, [0] * 5), (0x8130, 0x11, [1, 2, 3, 4, 5]),
                                   (0xFFFF, 0xFF, [0xFF] * 5))]
    emcy.append(wire.emcy(eds, 127, 0x8130, 0x11, [1, 2, 3, 4, 5]))

    sdo = []
    probes = [(i, s, eds.datatype(i, s if eds.has(i, s) else None)[0] // 8, eds.value(i, s if eds.has(i, s) else None))
              for i, s in ((0x1000, 0), (0x1017, 0), (0x1005, 0), (0x6060, 0), (0x6085, 0))]
    probes.append((0x6040, 0, 2, 0xFFFF))
    for n in NODES:
        for index, sub, size, value in probes:
            sdo.append(wire.sdo_upload_request(n, index, sub))
            sdo.append(wire.sdo_upload_response(n, index, sub, size, value))
            sdo.append(wire.sdo_download_request(n, index, sub, size, value))
            sdo.append(wire.sdo_download_response(n, index, sub))
        for code, _ in ABORTS:
            sdo.append(wire.sdo_abort(n, 0x6040, 0, code))
        sdo.append(wire.sdo_client_abort(n, 0x2011, 0, 0x08000020))
    sdo.append(wire.sdo_upload_request(127, 0x1000, 0))
    sdo.append(wire.sdo_abort(127, 0x1000, 0, 0x06020000))

    pdo = []
    for p in eds.pdos():
        for n in NODES:
            pdo += wire.pdo_vectors(p, n, full=True)
        pdo += wire.pdo_vectors(p, 127, full=False)

    return {"nmt": nmt, "heartbeat": heartbeat, "sync": syncs, "emcy": emcy, "sdo": sdo, "pdo": pdo}


def render(vectors):
    """One object per line keeps diffs reviewable and the output byte-stable."""
    return "[\n" + ",\n".join(" " + json.dumps(v, sort_keys=False, separators=(", ", ": "))
                              for v in vectors) + "\n]\n"


def generate(eds=None):
    return {f"{name}.json": render(vectors) for name, vectors in families(eds or wire.Eds()).items()}


def main(argv):
    out = pathlib.Path(argv[1]) if len(argv) > 1 else GOLDEN_DIR
    out.mkdir(parents=True, exist_ok=True)
    for name, text in generate().items():
        (out / name).write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
