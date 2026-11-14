# SPDX-License-Identifier: Apache-2.0
"""Load and validate a CiA 306 EDS of the virtual drive.

`load(path)` keeps every value as written, so a malformed value reaches `validate` as a finding
instead of stopping the load. `validate(od)` returns one message per defect, empty when clean.
"""
import configparser
import dataclasses
import pathlib
import re
from typing import NamedTuple, Optional

# CiA 301 basic data types: index -> (name, bits, signed).
DATA_TYPES = {
    0x0001: ("BOOLEAN", 1, False),
    0x0002: ("INTEGER8", 8, True),
    0x0003: ("INTEGER16", 16, True),
    0x0004: ("INTEGER32", 32, True),
    0x0005: ("UNSIGNED8", 8, False),
    0x0006: ("UNSIGNED16", 16, False),
    0x0007: ("UNSIGNED32", 32, False),
}

# CiA 306 access types and the PDO direction each one allows when PDOMapping=1.
ACCESS_TYPES = {"ro", "wo", "rw", "rwr", "rww", "const"}
RPDO_ACCESS = {"rww", "rw", "wo"}
TPDO_ACCESS = {"rwr", "rw", "ro", "const"}

MAX_PDO_BITS = 64
VAR, ARRAY, RECORD = 0x7, 0x8, 0x9

# A plain integer, optionally preceded by `$NODEID+`. Lely dcf-tools 2.3.5 parse only this
# order; `0x80+$NODEID` fails there (E3 gate, check 3), so it is rejected here as well.
_VALUE = re.compile(r"(?P<node>\$NODEID(?:\s*\+\s*)?)?(?P<num>-?(?:0x[0-9a-f]+|[0-9]+))?$", re.I)


class Value(NamedTuple):
    value: int
    node_relative: bool = False


def parse_value(raw: str) -> Value:
    m = _VALUE.match(raw.strip())
    if not m or not raw.strip():
        raise ValueError(f"invalid value {raw!r}")
    node = m.group("node")
    num = m.group("num")
    if num is None:
        if node is None or "+" in node:
            raise ValueError(f"invalid value {raw!r}")
        return Value(0, True)
    if node is not None and "+" not in node:
        raise ValueError(f"invalid value {raw!r}")
    return Value(int(num, 0), node is not None)


@dataclasses.dataclass
class Entry:
    index: int
    sub: int
    name: str
    data_type: Optional[int]
    access: str
    raw_default: str
    pdo_mapping: bool
    raw_low: Optional[str] = None
    raw_high: Optional[str] = None

    @property
    def where(self) -> str:
        return f"0x{self.index:04X}:{self.sub:02X}"

    @property
    def bits(self) -> int:
        return DATA_TYPES[self.data_type][1]

    @property
    def signed(self) -> bool:
        return DATA_TYPES[self.data_type][2]

    def default(self) -> Value:
        return parse_value(self.raw_default)

    def low(self) -> Optional[Value]:
        return None if self.raw_low is None else parse_value(self.raw_low)

    def high(self) -> Optional[Value]:
        return None if self.raw_high is None else parse_value(self.raw_high)


@dataclasses.dataclass
class Object:
    index: int
    name: str
    object_type: int
    sub_number: Optional[int]
    entries: dict = dataclasses.field(default_factory=dict)


class Mapped(NamedTuple):
    index: int
    sub: int
    bits: int


@dataclasses.dataclass
class Pdo:
    kind: str  # "rpdo" or "tpdo"
    number: int
    comm_index: int
    map_index: int
    cob_id: Optional[Value]
    transmission_type: Optional[int]
    mapping: list

    @property
    def label(self) -> str:
        return f"{self.kind.upper()}{self.number}"

    @property
    def bits(self) -> int:
        return sum(m.bits for m in self.mapping)


@dataclasses.dataclass
class ObjectDictionary:
    path: pathlib.Path
    objects: dict  # index -> Object
    listed: dict  # "MandatoryObjects" etc. -> list of indices
    dummy_usage: dict  # dummy index -> enabled
    problems: list  # structural defects found while loading

    def entry(self, index: int, sub: int) -> Optional[Entry]:
        obj = self.objects.get(index)
        return None if obj is None else obj.entries.get(sub)

    def entries(self):
        for index in sorted(self.objects):
            for sub in sorted(self.objects[index].entries):
                yield self.objects[index].entries[sub]

    def pdo(self, kind: str, number: int) -> Pdo:
        base = {"rpdo": 0x1400, "tpdo": 0x1800}[kind]
        comm, mapping = base + number - 1, base + number - 1 + 0x200
        if comm not in self.objects or mapping not in self.objects:
            raise KeyError(f"{kind.upper()}{number} not defined")
        cob = self.entry(comm, 1)
        tt = self.entry(comm, 2)
        return Pdo(kind, number, comm, mapping,
                   _try(lambda: cob.default()) if cob else None,
                   _try(lambda: tt.default().value) if tt else None,
                   self._mapping(mapping))

    def pdos(self):
        found = []
        for kind, base in (("rpdo", 0x1400), ("tpdo", 0x1800)):
            for number in range(1, 513):
                if base + number - 1 in self.objects and base + number - 1 + 0x200 in self.objects:
                    found.append(self.pdo(kind, number))
        return found

    def _mapping(self, index: int) -> list:
        count = _try(lambda: self.entry(index, 0).default().value)
        out = []
        for i in range(1, (count or 0) + 1):
            entry = self.entry(index, i)
            value = _try(lambda: entry.default().value) if entry else None
            if value is None:
                continue
            out.append(Mapped(value >> 16, (value >> 8) & 0xFF, value & 0xFF))
        return out


def _try(fn):
    try:
        return fn()
    except (ValueError, KeyError, AttributeError):
        return None


_SECTION = re.compile(r"([0-9A-F]{4})(?:SUB([0-9A-F]{1,2}))?$", re.I)


def load(path) -> ObjectDictionary:
    path = pathlib.Path(path)
    cfg = configparser.ConfigParser(interpolation=None, comment_prefixes=(";", "#"),
                                    inline_comment_prefixes=(";",))
    cfg.optionxform = str
    with path.open() as f:  # FileNotFoundError for a missing file
        cfg.read_file(f, str(path))

    sections = {}
    for name in cfg.sections():
        sections[name.upper()] = cfg[name]

    problems = []
    objects = {}
    subs = {}
    for name, section in sections.items():
        m = _SECTION.match(name)
        if not m:
            continue
        index = int(m.group(1), 16)
        if m.group(2) is None:
            objects[index] = Object(
                index, section.get("ParameterName", ""),
                _int(section.get("ObjectType", "0x7"), problems, f"[{name}] ObjectType"),
                _int(section["SubNumber"], problems, f"[{name}] SubNumber") if "SubNumber" in section else None)
        else:
            subs[(index, int(m.group(2), 16))] = section

    for (index, sub), section in subs.items():
        if index not in objects:
            problems.append(f"[{index:04X}sub{sub:X}] has no parent object 0x{index:04X}")
            continue
        objects[index].entries[sub] = _entry(index, sub, section, problems)
    for index, obj in objects.items():
        if obj.sub_number is None:
            obj.entries[0] = _entry(index, 0, sections[f"{index:04X}"], problems)

    listed = {}
    for list_name in ("MandatoryObjects", "OptionalObjects", "ManufacturerObjects"):
        section = sections.get(list_name.upper())
        listed[list_name] = [] if section is None else [
            _int(v, problems, f"[{list_name}] {k}") for k, v in section.items() if k != "SupportedObjects"]

    dummy_usage = {}
    for k, v in sections.get("DUMMYUSAGE", {}).items():
        m = re.fullmatch(r"Dummy([0-9A-F]{4})", k, re.I)
        if m:
            dummy_usage[int(m.group(1), 16)] = v.strip() == "1"

    return ObjectDictionary(path, objects, listed, dummy_usage, problems)


def _int(raw, problems, where):
    try:
        return int(raw, 0)
    except ValueError:
        problems.append(f"{where}: {raw!r} is not an integer")
        return None


def _entry(index, sub, section, problems) -> Entry:
    where = f"0x{index:04X}:{sub:02X}"
    return Entry(
        index, sub, section.get("ParameterName", ""),
        _int(section["DataType"], problems, f"{where} DataType") if "DataType" in section else None,
        section.get("AccessType", "").strip().lower(),
        section.get("DefaultValue", "0").strip() or "0",
        section.get("PDOMapping", "0").strip() == "1",
        section.get("LowLimit", "").strip() or None,
        section.get("HighLimit", "").strip() or None)


# Objects every drive EDS must define (CiA 301 mandatory objects plus the identity subs).
REQUIRED = [(0x1000, 0), (0x1001, 0), (0x1018, 0), (0x1018, 1), (0x1018, 2), (0x1018, 3), (0x1018, 4)]


def validate(od: ObjectDictionary) -> list:
    errors = list(od.problems)
    errors += _check_required(od)
    errors += _check_lists(od)
    for obj in od.objects.values():
        errors += _check_object(obj)
    for entry in od.entries():
        errors += _check_entry(entry)
    errors += _check_pdos(od)
    return errors


def _check_required(od):
    return [f"0x{i:04X}:{s:02X} is missing" for i, s in REQUIRED if od.entry(i, s) is None]


def _check_lists(od):
    errors = []
    listed = [i for indices in od.listed.values() for i in indices if i is not None]
    for index in sorted(set(listed) - set(od.objects)):
        errors.append(f"0x{index:04X} is listed but not defined")
    for index in sorted(set(od.objects) - set(listed)):
        errors.append(f"0x{index:04X} is defined but not listed in an object list")
    return errors


def _check_object(obj):
    errors = []
    if obj.sub_number is None:
        if obj.object_type != VAR:
            errors.append(f"0x{obj.index:04X} has no SubNumber but ObjectType {obj.object_type}")
        return errors
    if obj.object_type not in (ARRAY, RECORD):
        errors.append(f"0x{obj.index:04X} has sub-objects but ObjectType {obj.object_type}")
    if len(obj.entries) != obj.sub_number:
        errors.append(f"0x{obj.index:04X} declares SubNumber {obj.sub_number} "
                      f"but defines {len(obj.entries)} sub-objects")
    sub0 = obj.entries.get(0)
    if sub0 is None:
        errors.append(f"0x{obj.index:04X}:00 is missing")
    else:
        if sub0.data_type != 0x0005:
            errors.append(f"{sub0.where} must be UNSIGNED8")
        highest = _try(lambda: sub0.default().value)
        if highest is not None and highest != max(obj.entries):
            errors.append(f"{sub0.where} says highest sub-index {highest}, "
                          f"but the highest defined is {max(obj.entries)}")
    return errors


def _check_entry(entry):
    errors = []
    if entry.data_type not in DATA_TYPES:
        return [f"{entry.where} has unsupported DataType {entry.data_type}"]
    if entry.access not in ACCESS_TYPES:
        errors.append(f"{entry.where} has invalid AccessType {entry.access!r}")
    _, bits, signed = DATA_TYPES[entry.data_type]
    lo, hi = (-(1 << (bits - 1)), (1 << (bits - 1)) - 1) if signed else (0, (1 << bits) - 1)
    for label, raw in (("DefaultValue", entry.raw_default), ("LowLimit", entry.raw_low),
                       ("HighLimit", entry.raw_high)):
        if raw is None:
            continue
        try:
            value = parse_value(raw)
        except ValueError:
            hint = ""
            if "$NODEID" in raw.upper():
                hint = "; write '$NODEID+0x...' (the only node-relative form Lely dcf-tools parse)"
            errors.append(f"{entry.where} {label} {raw!r} is not a valid value{hint}")
            continue
        # A node-relative value must stay in range for every node-ID 1..127.
        low_end = value.value + (1 if value.node_relative else 0)
        high_end = value.value + (127 if value.node_relative else 0)
        if low_end < lo or high_end > hi:
            errors.append(f"{entry.where} {label} {raw!r} is out of range for "
                          f"{DATA_TYPES[entry.data_type][0]}")
    low, high = _try(entry.low), _try(entry.high)
    default = _try(entry.default)
    if default is not None and not default.node_relative:
        if low is not None and default.value < low.value:
            errors.append(f"{entry.where} DefaultValue is below LowLimit")
        if high is not None and default.value > high.value:
            errors.append(f"{entry.where} DefaultValue is above HighLimit")
    return errors


def _check_pdos(od):
    errors = []
    for comm in sorted(od.objects):
        if 0x1400 <= comm < 0x1600 or 0x1800 <= comm < 0x1A00:
            if comm + 0x200 not in od.objects:
                errors.append(f"0x{comm:04X} has no mapping object 0x{comm + 0x200:04X}")
        if 0x1600 <= comm < 0x1800 or 0x1A00 <= comm < 0x1C00:
            if comm - 0x200 not in od.objects:
                errors.append(f"0x{comm:04X} has no communication object 0x{comm - 0x200:04X}")
    cob_ids = {}
    for pdo in od.pdos():
        if pdo.cob_id is None:
            errors.append(f"{pdo.label} has no valid COB-ID at 0x{pdo.comm_index:04X}:01")
        else:
            cob_ids.setdefault(pdo.cob_id, []).append(pdo.label)
        if pdo.transmission_type is None:
            errors.append(f"{pdo.label} has no valid transmission type at 0x{pdo.comm_index:04X}:02")
        allowed = RPDO_ACCESS if pdo.kind == "rpdo" else TPDO_ACCESS
        for i, m in enumerate(pdo.mapping, 1):
            ref = f"0x{m.index:04X}:{m.sub:02X}"
            if m.index < 0x1000:
                if not od.dummy_usage.get(m.index, False):
                    errors.append(f"{pdo.label} entry {i} maps dummy {ref}, not enabled in [DummyUsage]")
                elif m.index not in DATA_TYPES or m.bits != DATA_TYPES[m.index][1]:
                    errors.append(f"{pdo.label} entry {i} maps dummy {ref} with {m.bits} bits")
                continue
            target = od.entry(m.index, m.sub)
            if target is None:
                errors.append(f"{pdo.label} entry {i} maps {ref}, which does not exist")
                continue
            if not target.pdo_mapping:
                errors.append(f"{pdo.label} entry {i} maps {ref}, which is not PDO-mappable")
            elif target.access not in allowed:
                errors.append(f"{pdo.label} entry {i} maps {ref} (access {target.access}), "
                              f"which is not {pdo.kind.upper()}-mappable")
            if target.data_type in DATA_TYPES and m.bits != target.bits:
                errors.append(f"{pdo.label} entry {i} maps {ref} with {m.bits} bits, "
                              f"but the object has {target.bits}")
        if pdo.bits > MAX_PDO_BITS:
            errors.append(f"{pdo.label} maps {pdo.bits} bits, more than {MAX_PDO_BITS}")
        if pdo.bits % 8:
            errors.append(f"{pdo.label} maps {pdo.bits} bits, not a whole number of bytes")
    for cob, labels in cob_ids.items():
        if len(labels) > 1:
            errors.append(f"{', '.join(labels)} share COB-ID {cob}")
    return errors
