# SPDX-License-Identifier: Apache-2.0
"""Deliberately broken copies of the drive EDS, one defect per copy.

Each defect edits a parsed copy of the real EDS and then checks that its edit changed what it
meant to change, so a renamed section can never turn a defect into a silent no-op.
"""
import configparser
import pathlib

EDS = pathlib.Path(__file__).resolve().parents[3] / "config" / "od" / "vdrive_arm.eds"


def _read(path=EDS):
    cfg = configparser.ConfigParser(interpolation=None, comment_prefixes=(";", "#"))
    cfg.optionxform = str
    cfg.read(path)
    return cfg


def _set(cfg, section, key, value):
    assert cfg.has_option(section, key), f"defect targets missing [{section}] {key}"
    assert cfg[section][key] != value, f"defect leaves [{section}] {key} unchanged"
    cfg[section][key] = value


def map_72_bits(cfg):
    # A fourth RPDO1 entry: an 8-bit UNSIGNED8 dummy after the 64 bits already mapped.
    _set(cfg, "1600", "SubNumber", "5")
    _set(cfg, "1600sub0", "DefaultValue", "4")
    assert not cfg.has_section("1600sub4")
    cfg["1600sub4"] = {
        "ParameterName": "RPDO1 mapping entry 4",
        "DataType": "0x0007",
        "AccessType": "ro",
        "DefaultValue": "0x00050008",
        "PDOMapping": "0",
    }
    assert not cfg.has_section("DummyUsage")
    cfg["DummyUsage"] = {"Dummy0005": "1"}


def unmappable_mapped(cfg):
    # TPDO1 maps 0x2030:05 drive_flags, which the EDS declares not PDO-mappable.
    _set(cfg, "1A00sub3", "DefaultValue", "0x20300510")


def read_only_in_rpdo(cfg):
    # RPDO1 maps 0x6064 position actual value (ro, TPDO-mappable only) instead of 0x607A.
    _set(cfg, "1600sub1", "DefaultValue", "0x60640020")


def missing_1018_sub1(cfg):
    assert cfg.remove_section("1018sub1"), "defect targets missing [1018sub1]"


def nodeid_suffix_form(cfg):
    # The form Lely dcf-tools 2.3.5 cannot parse (E3 gate, check 3).
    _set(cfg, "1014", "DefaultValue", "0x80+$NODEID")


# Each broken copy and the message od.validate must report for it.
MESSAGES = {
    "map_72_bits": r"RPDO1 maps 72 bits, more than 64",
    "unmappable_mapped": r"TPDO1 entry 3 maps 0x2030:05, which is not PDO-mappable",
    "read_only_in_rpdo": r"RPDO1 entry 1 maps 0x6064:00 \(access ro\), which is not RPDO-mappable",
    "missing_1018_sub1": r"0x1018:01 is missing",
    "nodeid_suffix_form": r"0x1014:00 DefaultValue '0x80\+\$NODEID'.*write '\$NODEID\+0x\.\.\.'",
}


DEFECTS = {
    "map_72_bits": map_72_bits,
    "unmappable_mapped": unmappable_mapped,
    "read_only_in_rpdo": read_only_in_rpdo,
    "missing_1018_sub1": missing_1018_sub1,
    "nodeid_suffix_form": nodeid_suffix_form,
}


def write_broken(name, directory):
    cfg = _read()
    DEFECTS[name](cfg)
    out = pathlib.Path(directory) / f"broken_{name}.eds"
    with out.open("w") as f:
        cfg.write(f)
    return out
