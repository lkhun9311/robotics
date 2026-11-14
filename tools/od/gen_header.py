#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate the C++ object dictionary header from the drive EDS.

    gen_header.py <eds> <out.hpp>

Every value in the header comes from the EDS; the generator refuses an EDS that `od.validate`
rejects and then writes nothing.
"""
import pathlib
import re
import sys

import od

NAMESPACE = "canopen_codec::od"
ACCESS = {"ro": "kRo", "wo": "kWo", "rw": "kRw", "rwr": "kRwr", "rww": "kRww", "const": "kConst"}


def snake(text: str) -> str:
    return re.sub(r"[^0-9a-z]+", "_", text.lower()).strip("_")


def cpp_type(entry: od.Entry) -> str:
    return f"std::{'' if entry.signed else 'u'}int{entry.bits}_t"


def _hex(value: int, width: int) -> str:
    return f"0x{value:0{width}X}"


def _address_name(dictionary, entry):
    obj = dictionary.objects[entry.index]
    if obj.sub_number is None:
        return snake(entry.name)
    return f"{snake(obj.name)}_{snake(entry.name)}"


def _limit(value):
    if value is None:
        return "false, 0"
    return f"true, {value.value}"


def render(dictionary: od.ObjectDictionary, source_name: str) -> str:
    lines = [
        "// SPDX-License-Identifier: Apache-2.0",
        f"// Generated from {source_name} by tools/od/gen_header.py. Do not edit.",
        "#pragma once",
        "",
        "#include <array>",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        f"namespace {NAMESPACE} {{",
        "",
        "enum class Access : std::uint8_t { kRo, kWo, kRw, kRwr, kRww, kConst };",
        "",
        "struct Address {",
        "  std::uint16_t index;",
        "  std::uint8_t sub;",
        "};",
        "",
        "// One object dictionary entry. A default or limit that adds the node-ID has",
        "// default_adds_node_id set; the stored value is the part before the node-ID is added.",
        "struct EntryInfo {",
        "  std::uint16_t index;",
        "  std::uint8_t sub;",
        "  std::uint8_t size;",
        "  bool is_signed;",
        "  Access access;",
        "  bool pdo_mappable;",
        "  std::int64_t default_value;",
        "  bool default_adds_node_id;",
        "  bool has_low;",
        "  std::int64_t low;",
        "  bool has_high;",
        "  std::int64_t high;",
        "};",
        "",
        "// One field of a PDO, in mapping order; offset and size in bytes, little-endian.",
        "struct Field {",
        "  const char* name;",
        "  std::uint16_t index;",
        "  std::uint8_t sub;",
        "  std::uint8_t offset;",
        "  std::uint8_t size;",
        "  bool is_signed;",
        "};",
        "",
    ]

    sync = dictionary.entry(0x1005, 0).default()
    emcy = dictionary.entry(0x1014, 0).default()
    if sync.node_relative or not emcy.node_relative:
        raise ValueError("expected an absolute SYNC COB-ID (0x1005) and a node-relative EMCY COB-ID (0x1014)")
    # Flag bits (31 invalid, 30 SYNC producer, 29 extended frame) are not part of an identifier;
    # refuse them instead of masking them away.
    for label, value in (("SYNC COB-ID 0x1005", sync), ("EMCY COB-ID 0x1014", emcy)):
        if value.value & ~0x7FF:
            raise ValueError(f"{label} {value.value:#x} is not a plain 11-bit identifier")
    lines += [
        "// Service identifiers (0x1005 SYNC, 0x1014 EMCY; EMCY adds the node-ID).",
        f"inline constexpr std::uint16_t kSyncCobId = {_hex(sync.value, 3)};",
        f"inline constexpr std::uint16_t kEmcyCobIdBase = {_hex(emcy.value, 3)};",
        "",
        "namespace addr {",
    ]
    names = set()
    for entry in dictionary.entries():
        name = _address_name(dictionary, entry)
        if name in names:
            raise ValueError(f"two entries map to the C++ name {name}")
        names.add(name)
        lines.append(f"inline constexpr Address {name}{{{_hex(entry.index, 4)}, {_hex(entry.sub, 2)}}};")
    lines += ["}  // namespace addr", ""]

    rows = []
    for entry in dictionary.entries():
        default = entry.default()
        rows.append(
            f"    {{{_hex(entry.index, 4)}, {_hex(entry.sub, 2)}, {entry.bits // 8}, "
            f"{'true' if entry.signed else 'false'}, Access::{ACCESS[entry.access]}, "
            f"{'true' if entry.pdo_mapping else 'false'}, {default.value}, "
            f"{'true' if default.node_relative else 'false'}, {_limit(entry.low())}, {_limit(entry.high())}}},")
    lines += [f"inline constexpr std::array<EntryInfo, {len(rows)}> kEntries{{{{", *rows, "}};", ""]

    for pdo in dictionary.pdos():
        lines += _render_pdo(dictionary, pdo)

    lines += [f"}}  // namespace {NAMESPACE}", ""]
    return "\n".join(lines)


def _render_pdo(dictionary, pdo):
    if pdo.cob_id.value & ~0x7FF:
        raise ValueError(f"{pdo.label} COB-ID {pdo.cob_id.value:#x} is not a valid 11-bit default")
    name = f"{pdo.kind.capitalize()}{pdo.number}"
    fields, members, offset, seen = [], [], 0, set()
    for m in pdo.mapping:
        entry = dictionary.entry(m.index, m.sub)
        field = snake(entry.name)
        if field in seen:
            raise ValueError(f"{pdo.label} maps two fields named {field}")
        seen.add(field)
        size = m.bits // 8
        fields.append(f'      {{"{field}", {_hex(m.index, 4)}, {_hex(m.sub, 2)}, {offset}, {size}, '
                      f"{'true' if entry.signed else 'false'}}},")
        members.append(f"  {cpp_type(entry)} {field}{{}};")
        offset += size
    return [
        f"// {pdo.label}: communication 0x{pdo.comm_index:04X}, mapping 0x{pdo.map_index:04X}.",
        f"struct {name} {{",
        f"  static constexpr std::uint16_t kCobIdBase = {_hex(pdo.cob_id.value, 3)};",
        f"  static constexpr bool kCobIdAddsNodeId = {'true' if pdo.cob_id.node_relative else 'false'};",
        f"  static constexpr std::uint8_t kTransmissionType = {pdo.transmission_type};",
        f"  static constexpr std::size_t kLength = {offset};",
        f"  static constexpr std::array<Field, {len(fields)}> kFields{{{{",
        *fields,
        "  }};",
        *members,
        "};",
        "",
    ]


def main(argv) -> int:
    if len(argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2
    eds, out = pathlib.Path(argv[1]), pathlib.Path(argv[2])
    dictionary = od.load(eds)
    errors = od.validate(dictionary)
    if errors:
        for error in errors:
            print(f"{eds}: {error}", file=sys.stderr)
        return 1
    try:
        text = render(dictionary, eds.name)
    except ValueError as error:
        print(f"{eds}: {error}", file=sys.stderr)
        return 1
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(out.suffix + ".tmp")
    tmp.write_text(text)
    tmp.replace(out)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
