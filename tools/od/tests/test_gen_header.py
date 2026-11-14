# SPDX-License-Identifier: Apache-2.0
"""The generated C++ header against layouts computed here, directly from the EDS text.

The expected side reads the EDS with configparser only (no od.py), so a bug shared by the
loader and the generator cannot hide itself.
"""
import configparser
import pathlib
import re
import shutil
import subprocess
import sys

import pytest

import defects
import gen_header

SCRIPT = pathlib.Path(gen_header.__file__)
SIGNED = {0x0002, 0x0003, 0x0004}
CPP_TYPE = {(1, False): "std::uint8_t", (2, False): "std::uint16_t", (4, False): "std::uint32_t",
            (1, True): "std::int8_t", (2, True): "std::int16_t", (4, True): "std::int32_t"}
PDOS = {"Rpdo1": (0x1400, 0x1600), "Rpdo2": (0x1401, 0x1601), "Rpdo3": (0x1402, 0x1602),
        "Tpdo1": (0x1800, 0x1A00), "Tpdo2": (0x1801, 0x1A01), "Tpdo3": (0x1802, 0x1A02)}


def _raw():
    cfg = configparser.ConfigParser(interpolation=None, comment_prefixes=(";", "#"))
    cfg.optionxform = str
    cfg.read(defects.EDS)
    return {s.upper(): cfg[s] for s in cfg.sections()}


def _section(raw, index, sub):
    name = f"{index:04X}SUB{sub:X}"
    return raw[name] if name in raw else raw[f"{index:04X}"]


def _snake(text):
    return re.sub(r"[^0-9a-z]+", "_", text.lower()).strip("_")


def _expected_layout(raw, map_index):
    fields, offset = [], 0
    for i in range(1, int(raw[f"{map_index:04X}SUB0"]["DefaultValue"], 0) + 1):
        value = int(raw[f"{map_index:04X}SUB{i:X}"]["DefaultValue"], 0)
        index, sub, bits = value >> 16, (value >> 8) & 0xFF, value & 0xFF
        target = _section(raw, index, sub)
        signed = int(target["DataType"], 0) in SIGNED
        fields.append((_snake(target["ParameterName"]), index, sub, offset, bits // 8, signed))
        offset += bits // 8
    return fields, offset


def _strip_node(raw_value):
    assert raw_value.startswith("$NODEID+"), raw_value
    return int(raw_value[len("$NODEID+"):], 0)


@pytest.fixture(scope="module")
def header(tmp_path_factory):
    out = tmp_path_factory.mktemp("gen") / "vdrive_od.hpp"
    subprocess.run([sys.executable, str(SCRIPT), str(defects.EDS), str(out)], check=True)
    return out.read_text()


def _struct(header, name):
    m = re.search(rf"^struct {name} \{{\n(.*?)^\}};", header, re.S | re.M)
    assert m, f"struct {name} not in header"
    return m.group(1)


@pytest.mark.parametrize("name", sorted(PDOS))
def test_pdo_layout_matches_eds_maps(header, name):
    raw = _raw()
    comm, mapping = PDOS[name]
    fields, length = _expected_layout(raw, mapping)
    body = _struct(header, name)
    got = [(f, int(i, 16), int(s, 16), int(o), int(z), b == "true") for f, i, s, o, z, b in re.findall(
        r'\{"(\w+)", 0x([0-9A-F]{4}), 0x([0-9A-F]{2}), (\d+), (\d+), (true|false)\}', body)]
    assert got == fields
    assert re.search(rf"kLength = {length};", body)
    base = _strip_node(raw[f"{comm:04X}SUB1"]["DefaultValue"])
    assert re.search(rf"kCobIdBase = 0x{base:03X};", body)
    assert re.search(r"kCobIdAddsNodeId = true;", body)
    tt = int(raw[f"{comm:04X}SUB2"]["DefaultValue"], 0)
    assert re.search(rf"kTransmissionType = {tt};", body)
    members = re.findall(r"^  (std::u?int\d+_t) (\w+)\{\};", body, re.M)
    assert members == [(CPP_TYPE[(size, signed)], f) for f, _, _, _, size, signed in fields]


def test_service_cob_ids(header):
    raw = _raw()
    sync = int(raw["1005"]["DefaultValue"], 0)
    emcy = _strip_node(raw["1014"]["DefaultValue"])
    assert re.search(rf"kSyncCobId = 0x{sync:03X};", header)
    assert re.search(rf"kEmcyCobIdBase = 0x{emcy:03X};", header)


def test_every_eds_entry_has_an_address_and_a_table_row(header):
    raw = _raw()
    rows = set(re.findall(r"\{0x([0-9A-F]{4}), 0x([0-9A-F]{2}), \d+, (?:true|false), Access::", header))
    expected = set()
    for name, section in raw.items():
        m = re.fullmatch(r"([0-9A-F]{4})(?:SUB([0-9A-F]+))?", name)
        if m and "DataType" in section and not (m.group(2) is None and "SubNumber" in section):
            expected.add((m.group(1), f"{int(m.group(2) or '0', 16):02X}"))
    assert rows == expected


def test_header_compiles_under_project_warnings(header, tmp_path):
    compiler = shutil.which("g++")
    assert compiler is not None, "g++ not on PATH"
    (tmp_path / "vdrive_od.hpp").write_text(header)
    unit = tmp_path / "use.cpp"
    unit.write_text('#include "vdrive_od.hpp"\n'
                    "static_assert(canopen_codec::od::Rpdo1::kFields.size() == 3);\n"
                    "int main() { canopen_codec::od::Rpdo1 r{}; return r.exec_tag; }\n")
    result = subprocess.run([compiler, "-std=c++20", "-Wall", "-Wextra", "-Wpedantic", "-Wconversion",
                             "-Wsign-conversion", "-Werror", "-fsyntax-only", str(unit)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_generator_refuses_an_invalid_eds(tmp_path):
    broken = defects.write_broken("read_only_in_rpdo", tmp_path)
    out = tmp_path / "out.hpp"
    result = subprocess.run([sys.executable, str(SCRIPT), str(broken), str(out)],
                            capture_output=True, text=True)
    assert result.returncode != 0
    assert "not RPDO-mappable" in result.stderr
    assert not out.exists()


@pytest.mark.parametrize("flags", [0x40000000, 0x80000000], ids=["producer_bit30", "invalid_bit31"])
def test_generator_refuses_flag_bits_in_the_sync_cob_id(tmp_path, flags):
    cfg = defects._read()
    cfg["1005"]["DefaultValue"] = f"0x{flags | 0x80:08X}"
    broken = tmp_path / "sync_flags.eds"
    with broken.open("w") as f:
        cfg.write(f)
    out = tmp_path / "out.hpp"
    result = subprocess.run([sys.executable, str(SCRIPT), str(broken), str(out)],
                            capture_output=True, text=True)
    assert result.returncode == 1, result.stderr
    assert "SYNC COB-ID 0x1005" in result.stderr and "not a plain 11-bit identifier" in result.stderr
    assert not out.exists()
