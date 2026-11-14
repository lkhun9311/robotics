# SPDX-License-Identifier: Apache-2.0
"""Independent CANopen wire encoder for the drive, computed only from the EDS.

This module deliberately imports neither od.py nor gen_header.py: those two also feed the C++
header generator, so sharing them would let one misparse make the C++ codec and the golden
vectors agree while both are wrong. The EDS is read here with configparser, and only what the
frames need: PDO communication and mapping, the data types of mapped objects, 0x1005, 0x1014.
All multi-byte values are little-endian, as in CiA 301.
"""
import configparser
import pathlib
import re

DEFAULT_EDS = pathlib.Path(__file__).resolve().parents[2] / "config" / "od" / "vdrive_arm.eds"

# CiA 301 data type code -> (bits, signed).
TYPES = {0x0002: (8, True), 0x0003: (16, True), 0x0004: (32, True),
         0x0005: (8, False), 0x0006: (16, False), 0x0007: (32, False)}

# Predefined connection set bases that the EDS does not carry.
NMT_COB = 0x000
HEARTBEAT_BASE = 0x700
SDO_REQUEST_BASE = 0x600
SDO_RESPONSE_BASE = 0x580

NMT_COMMANDS = {"start": 0x01, "stop": 0x02, "pre_operational": 0x80,
                "reset_node": 0x81, "reset_communication": 0x82}
HEARTBEAT_STATES = {"boot_up": 0x00, "stopped": 0x04, "operational": 0x05, "pre_operational": 0x7F}


def snake(text):
    return re.sub(r"[^0-9a-z]+", "_", text.lower()).strip("_")


def _number(text, node):
    """Parse '100', '0x80' or '$NODEID+0x200' (node-relative) to an int."""
    text = text.strip()
    if text.startswith("$NODEID"):
        rest = text[len("$NODEID"):].lstrip("+")
        return node + (int(rest, 0) if rest else 0)
    return int(text, 0)


class Eds:
    def __init__(self, path=DEFAULT_EDS):
        self._cp = configparser.ConfigParser(interpolation=None, strict=True)
        self._cp.optionxform = str
        with open(path, encoding="utf-8") as handle:
            self._cp.read_file(handle)

    def _section(self, index, sub=None):
        name = f"{index:04X}" if sub is None else f"{index:04X}sub{sub}"
        if name not in self._cp:
            raise KeyError(f"EDS has no [{name}]")
        return self._cp[name]

    def value(self, index, sub=None, node=0):
        return _number(self._section(index, sub)["DefaultValue"], node)

    def has(self, index, sub=None):
        name = f"{index:04X}" if sub is None else f"{index:04X}sub{sub}"
        return name in self._cp

    def datatype(self, index, sub=None):
        return TYPES[int(self._section(index, sub)["DataType"], 0)]

    def name(self, index, sub=None):
        return self._section(index, sub)["ParameterName"]

    def sync_cob_id(self):
        return self.value(0x1005) & 0x7FF

    def emcy_cob_id(self, node):
        return self.value(0x1014, node=node) & 0x7FF

    def pdos(self):
        """Yield PDO descriptions in a stable order: RPDO1..n then TPDO1..n."""
        for kind, comm_base, map_base in (("rpdo", 0x1400, 0x1600), ("tpdo", 0x1800, 0x1A00)):
            for number in range(1, 513):
                comm = comm_base + number - 1
                if not self.has(comm):
                    break
                mapping = map_base + number - 1
                fields, offset = [], 0
                for sub in range(1, self.value(mapping, 0) + 1):
                    word = self.value(mapping, sub)
                    index, osub, bits = word >> 16, (word >> 8) & 0xFF, word & 0xFF
                    has_sub = self.has(index, osub)
                    entry = (index, osub) if has_sub else (index, None)
                    width, signed = self.datatype(*entry)
                    if width != bits:
                        raise ValueError(f"{kind}{number}: mapping says {bits} bits, type is {width}")
                    fields.append({"name": snake(self.name(*entry)), "index": index, "sub": osub,
                                   "bits": bits, "signed": signed, "offset": offset})
                    offset += bits // 8
                yield {"label": f"{kind.upper()}{number}", "comm": comm, "mapping": mapping,
                       "cob": self._section(comm, 1)["DefaultValue"],
                       "transmission_type": self.value(comm, 2), "fields": fields, "length": offset}


def pdo_cob_id(pdo, node):
    return _number(pdo["cob"], node) & 0x7FF


def le(value, bits):
    """Little-endian two's-complement bytes of value in the given width."""
    if not -(1 << (bits - 1)) <= value < (1 << bits):
        raise ValueError(f"{value} does not fit {bits} bits")
    return list((value & ((1 << bits) - 1)).to_bytes(bits // 8, "little"))


def frame(name, node, cob_id, data, fields=None):
    return {"name": name, "node": node, "cob_id": cob_id, "dlc": len(data), "data": list(data),
            "fields": dict(fields or {})}


def nmt(command, node):
    code = NMT_COMMANDS[command]
    return frame(f"nmt_{command}_node{node}", node, NMT_COB, [code, node],
                 {"command": code, "target_node": node})


def heartbeat(state, node):
    code = HEARTBEAT_STATES[state]
    return frame(f"heartbeat_{state}_node{node}", node, HEARTBEAT_BASE + node, [code], {"state": code})


def sync(eds):
    return frame("sync", 0, eds.sync_cob_id(), [])


def emcy(eds, node, error_code, error_register, manufacturer):
    data = le(error_code, 16) + [error_register] + list(manufacturer)
    return frame(f"emcy_{error_code:04x}_node{node}", node, eds.emcy_cob_id(node), data,
                 {"error_code": error_code, "error_register": error_register,
                  "manufacturer": int.from_bytes(bytes(manufacturer), "little")})


def _sdo_head(index, sub):
    return le(index, 16) + [sub]


def sdo_upload_request(node, index, sub):
    return frame(f"sdo_upload_request_{index:04x}_{sub:02x}_node{node}", node, SDO_REQUEST_BASE + node,
                 [0x40] + _sdo_head(index, sub) + [0] * 4, {"index": index, "sub": sub})


def sdo_upload_response(node, index, sub, size, value):
    command = 0x43 | ((4 - size) << 2)
    data = [command] + _sdo_head(index, sub) + (le(value, size * 8) + [0] * 4)[:4]
    return frame(f"sdo_upload_response_{index:04x}_{sub:02x}_node{node}", node, SDO_RESPONSE_BASE + node,
                 data, {"index": index, "sub": sub, "size": size, "value": value})


def sdo_download_request(node, index, sub, size, value):
    command = 0x23 | ((4 - size) << 2)
    data = [command] + _sdo_head(index, sub) + (le(value, size * 8) + [0] * 4)[:4]
    return frame(f"sdo_download_request_{index:04x}_{sub:02x}_node{node}", node, SDO_REQUEST_BASE + node,
                 data, {"index": index, "sub": sub, "size": size, "value": value})


def sdo_download_response(node, index, sub):
    return frame(f"sdo_download_response_{index:04x}_{sub:02x}_node{node}", node, SDO_RESPONSE_BASE + node,
                 [0x60] + _sdo_head(index, sub) + [0] * 4, {"index": index, "sub": sub})


def sdo_abort(node, index, sub, code):
    return frame(f"sdo_abort_{code:08x}_{index:04x}_{sub:02x}_node{node}", node, SDO_RESPONSE_BASE + node,
                 [0x80] + _sdo_head(index, sub) + le(code, 32),
                 {"index": index, "sub": sub, "abort_code": code})


def pdo_frame(pdo, node, values, tag):
    data = []
    for field in pdo["fields"]:
        data += le(values[field["name"]], field["bits"])
    return frame(f"{pdo['label'].lower()}_{tag}_node{node}", node, pdo_cob_id(pdo, node), data, values)


def field_range(field):
    bits = field["bits"]
    return (-(1 << (bits - 1)), (1 << (bits - 1)) - 1) if field["signed"] else (0, (1 << bits) - 1)


def pdo_vectors(pdo, node, full):
    """Edge-value frames for one PDO. full=False gives just the all-maximum case."""
    fields = pdo["fields"]
    low = {f["name"]: field_range(f)[0] for f in fields}
    high = {f["name"]: field_range(f)[1] for f in fields}
    out = [pdo_frame(pdo, node, high, "all_max")]
    if not full:
        return out
    out.append(pdo_frame(pdo, node, {f["name"]: 0 for f in fields}, "all_zero"))
    out.append(pdo_frame(pdo, node, low, "all_min"))
    out.append(pdo_frame(pdo, node, {f["name"]: i + 1 for i, f in enumerate(fields)}, "ordered"))
    for f in fields:
        for tag, edge in (("min", low[f["name"]]), ("zero", 0), ("one", 1), ("max", high[f["name"]])):
            values = {g["name"]: i + 1 for i, g in enumerate(fields)}
            values[f["name"]] = edge
            out.append(pdo_frame(pdo, node, values, f"{f['name']}_{tag}"))
    return out
