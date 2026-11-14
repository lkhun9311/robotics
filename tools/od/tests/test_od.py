# SPDX-License-Identifier: Apache-2.0
"""The drive object dictionary against the E3a wire contract (D4, D10-D14, D17, D18).

The expected values below restate the contract by hand on purpose: they are the independent
side of the comparison, so the EDS cannot drift from the contract without a test failing.
"""
import re

import pytest

import defects
import od

U8, U16, U32 = 0x0005, 0x0006, 0x0007
I8, I16, I32 = 0x0002, 0x0003, 0x0004
NODE = "node"  # marks a default written relative to $NODEID

# (index, sub) -> (data type, access, default, PDO-mappable). A default (value, NODE) means
# `$NODEID+value`.
EXPECTED = {
    0x1000: {0: (U32, "ro", 0x00020192, False)},
    0x1001: {0: (U8, "ro", 0, False)},
    0x1005: {0: (U32, "rw", 0x80, False)},
    0x1014: {0: (U32, "ro", (0x80, NODE), False)},
    0x1017: {0: (U16, "rw", 100, False)},
    0x1018: {
        0: (U8, "ro", 4, False),
        1: (U32, "ro", 0, False),
        2: (U32, "ro", 0, False),
        3: (U32, "ro", 0, False),
        4: (U32, "ro", 0, False),
    },
    0x1400: {0: (U8, "ro", 2, False), 1: (U32, "rw", (0x200, NODE), False), 2: (U8, "rw", 1, False)},
    0x1401: {0: (U8, "ro", 2, False), 1: (U32, "rw", (0x300, NODE), False), 2: (U8, "rw", 1, False)},
    0x1402: {0: (U8, "ro", 2, False), 1: (U32, "rw", (0x400, NODE), False), 2: (U8, "rw", 255, False)},
    0x1600: {
        0: (U8, "ro", 3, False),
        1: (U32, "ro", 0x607A0020, False),
        2: (U32, "ro", 0x20200110, False),
        3: (U32, "ro", 0x20200210, False),
    },
    0x1601: {
        0: (U8, "ro", 3, False),
        1: (U32, "ro", 0x20210120, False),
        2: (U32, "ro", 0x20210210, False),
        3: (U32, "ro", 0x20210310, False),
    },
    0x1602: {0: (U8, "ro", 1, False), 1: (U32, "ro", 0x60400010, False)},
    0x1800: {0: (U8, "ro", 2, False), 1: (U32, "rw", (0x180, NODE), False), 2: (U8, "rw", 1, False)},
    0x1801: {0: (U8, "ro", 2, False), 1: (U32, "rw", (0x280, NODE), False), 2: (U8, "rw", 1, False)},
    0x1802: {0: (U8, "ro", 2, False), 1: (U32, "rw", (0x380, NODE), False), 2: (U8, "rw", 1, False)},
    0x1A00: {
        0: (U8, "ro", 3, False),
        1: (U32, "ro", 0x60410010, False),
        2: (U32, "ro", 0x60640020, False),
        3: (U32, "ro", 0x20300110, False),
    },
    0x1A01: {
        0: (U8, "ro", 3, False),
        1: (U32, "ro", 0x606C0020, False),
        2: (U32, "ro", 0x20300110, False),
        3: (U32, "ro", 0x20300210, False),
    },
    0x1A02: {
        0: (U8, "ro", 3, False),
        1: (U32, "ro", 0x20300320, False),
        2: (U32, "ro", 0x20300110, False),
        3: (U32, "ro", 0x20300410, False),
    },
    # D14, D17
    0x2010: {
        0: (U8, "ro", 4, False),
        1: (U32, "rw", 0, False),
        2: (U32, "rw", 0, False),
        3: (U32, "rw", 0, False),
        4: (U32, "rw", 0, False),
    },
    0x2011: {0: (U32, "rw", 0, False)},
    0x2012: {0: (U16, "ro", 0, False)},
    0x2013: {0: (U32, "ro", 0, False)},
    0x2014: {0: (U16, "ro", 30, False)},
    0x2015: {0: (U8, "rw", 0, False)},
    0x2016: {0: (U32, "ro", 0, False)},
    0x2020: {0: (U8, "ro", 2, False), 1: (U16, "rww", 0, True), 2: (U16, "rww", 0, True)},
    0x2021: {
        0: (U8, "ro", 3, False),
        1: (U32, "rww", 0, True),
        2: (U16, "rww", 0, True),
        3: (U16, "rww", 0, True),
    },
    0x2030: {
        0: (U8, "ro", 5, False),
        1: (U16, "ro", 0, True),
        2: (U16, "ro", 0, True),
        3: (U32, "ro", 0, True),
        4: (U16, "ro", 0, True),
        5: (U16, "ro", 0, False),
    },
    # CiA 402; D2 units, D10/D11 stop values
    0x6040: {0: (U16, "rww", 0, True)},
    0x6041: {0: (U16, "ro", 0, True)},
    0x605A: {0: (I16, "rw", 6, False)},
    0x605E: {0: (I16, "rw", 2, False)},
    0x6060: {0: (I8, "rw", 8, False)},
    0x6061: {0: (I8, "ro", 8, False)},
    0x6064: {0: (I32, "ro", 0, True)},
    0x606C: {0: (I32, "ro", 0, True)},
    0x607A: {0: (I32, "rww", 0, True)},
    0x6085: {0: (U32, "rw", 10_000_000, False)},
}

# Value limits (LowLimit, HighLimit) the drive enforces on SDO writes; every other entry has
# none. 0x6060: cyclic synchronous position is the only mode; 0x2015: policy_reset is 0 or 1.
LIMITS = {
    (0x6060, 0): (8, 8),
    (0x2015, 0): (0, 1),
}

# D4 and D13: (kind, number) -> (COB-ID base added to the node-ID, transmission type,
# mapping as (index, sub, bits) in order, total bits).
PDOS = {
    ("rpdo", 1): (0x200, 1, [(0x607A, 0, 32), (0x2020, 1, 16), (0x2020, 2, 16)], 64),
    ("rpdo", 2): (0x300, 1, [(0x2021, 1, 32), (0x2021, 2, 16), (0x2021, 3, 16)], 64),
    ("rpdo", 3): (0x400, 255, [(0x6040, 0, 16)], 16),
    ("tpdo", 1): (0x180, 1, [(0x6041, 0, 16), (0x6064, 0, 32), (0x2030, 1, 16)], 64),
    ("tpdo", 2): (0x280, 1, [(0x606C, 0, 32), (0x2030, 1, 16), (0x2030, 2, 16)], 64),
    ("tpdo", 3): (0x380, 1, [(0x2030, 3, 32), (0x2030, 1, 16), (0x2030, 4, 16)], 64),
}

@pytest.fixture(scope="module")
def dictionary():
    return od.load(defects.EDS)


def test_eds_validates_clean(dictionary):
    assert od.validate(dictionary) == []


def test_eds_holds_exactly_the_contract_objects(dictionary):
    assert sorted(dictionary.objects) == sorted(EXPECTED)


@pytest.mark.parametrize("index", sorted(EXPECTED), ids=lambda i: f"0x{i:04X}")
def test_object_entries(dictionary, index):
    obj = dictionary.objects[index]
    assert sorted(obj.entries) == sorted(EXPECTED[index])
    for sub, (data_type, access, default, mappable) in EXPECTED[index].items():
        entry = obj.entries[sub]
        where = f"0x{index:04X}:{sub:02X}"
        assert entry.data_type == data_type, where
        assert entry.access == access, where
        assert entry.pdo_mapping is mappable, where
        if isinstance(default, tuple):
            assert entry.default() == od.Value(default[0], node_relative=True), where
        else:
            assert entry.default() == od.Value(default, node_relative=False), where
        low, high = LIMITS.get((index, sub), (None, None))
        assert entry.low() == (None if low is None else od.Value(low)), where
        assert entry.high() == (None if high is None else od.Value(high)), where


@pytest.mark.parametrize("key", sorted(PDOS), ids=lambda k: f"{k[0]}{k[1]}")
def test_pdo_matches_contract(dictionary, key):
    base, transmission_type, mapping, bits = PDOS[key]
    pdo = dictionary.pdo(*key)
    assert pdo.cob_id == od.Value(base, node_relative=True)
    assert pdo.transmission_type == transmission_type
    assert [(m.index, m.sub, m.bits) for m in pdo.mapping] == mapping
    assert pdo.bits == bits


@pytest.mark.parametrize("key", sorted(PDOS), ids=lambda k: f"{k[0]}{k[1]}")
def test_pdo_maps_only_entries_mappable_in_its_direction(dictionary, key):
    allowed = {"rpdo": {"rww", "rw", "wo"}, "tpdo": {"ro", "rwr", "rw", "const"}}[key[0]]
    for m in dictionary.pdo(*key).mapping:
        entry = dictionary.objects[m.index].entries[m.sub]
        assert entry.pdo_mapping, f"0x{m.index:04X}:{m.sub:02X}"
        assert entry.access in allowed, f"0x{m.index:04X}:{m.sub:02X}"


def test_rpdo3_is_two_bytes_and_event_driven(dictionary):
    pdo = dictionary.pdo("rpdo", 3)
    assert pdo.bits // 8 == 2
    assert pdo.transmission_type == 255


def test_service_cob_ids(dictionary):
    assert dictionary.objects[0x1005].entries[0].default() == od.Value(0x80, node_relative=False)
    assert dictionary.objects[0x1014].entries[0].default() == od.Value(0x80, node_relative=True)


@pytest.mark.parametrize("raw, value", [
    ("0", od.Value(0, False)),
    ("30", od.Value(30, False)),
    ("0x00020192", od.Value(0x00020192, False)),
    ("-6", od.Value(-6, False)),
    ("$NODEID+0x180", od.Value(0x180, True)),
    ("$NODEID", od.Value(0, True)),
])
def test_parse_value_accepts_lely_forms(raw, value):
    assert od.parse_value(raw) == value


@pytest.mark.parametrize("raw", ["0x80+$NODEID", "", "0x", "12abc", "$NODEID-1"])
def test_parse_value_rejects_other_forms(raw):
    with pytest.raises(ValueError):
        od.parse_value(raw)


@pytest.mark.parametrize("name", sorted(defects.MESSAGES))
def test_validate_reports_defect(tmp_path, name):
    errors = od.validate(od.load(defects.write_broken(name, tmp_path)))
    assert any(re.search(defects.MESSAGES[name], e) for e in errors), errors


def test_every_defect_has_a_message():
    assert sorted(defects.MESSAGES) == sorted(defects.DEFECTS)


def test_load_fails_on_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        od.load(tmp_path / "absent.eds")
