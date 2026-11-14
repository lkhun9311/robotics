# SPDX-License-Identifier: Apache-2.0
"""The independent wire encoder against hand-written CiA 301 frames and the committed golden files.

The hand-written frames below are the check on wire.py itself; the golden files are checked
for freshness, shape and consistency with the EDS mapping.
"""
import json
import pathlib

import pytest

import make_golden
import wire

EDS = wire.Eds()


def test_sdo_upload_request_of_device_type():
    f = wire.sdo_upload_request(3, 0x1000, 0)
    assert (f["cob_id"], f["dlc"], f["data"]) == (0x603, 8, [0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0])


def test_nmt_start_node():
    f = wire.nmt("start", 5)
    assert (f["cob_id"], f["dlc"], f["data"]) == (0x000, 2, [0x01, 0x05])


def test_nmt_other_commands():
    assert wire.nmt("stop", 2)["data"] == [0x02, 0x02]
    assert wire.nmt("pre_operational", 2)["data"] == [0x80, 0x02]
    assert wire.nmt("reset_node", 2)["data"] == [0x81, 0x02]
    assert wire.nmt("reset_communication", 2)["data"] == [0x82, 0x02]


@pytest.mark.parametrize("state,byte", [("boot_up", 0x00), ("stopped", 0x04), ("operational", 0x05),
                                        ("pre_operational", 0x7F)])
def test_heartbeat_states(state, byte):
    f = wire.heartbeat(state, 6)
    assert (f["cob_id"], f["dlc"], f["data"]) == (0x706, 1, [byte])


def test_sdo_abort_object_does_not_exist():
    f = wire.sdo_abort(4, 0x1234, 0x56, 0x06020000)
    assert (f["cob_id"], f["data"]) == (0x584, [0x80, 0x34, 0x12, 0x56, 0x00, 0x00, 0x02, 0x06])


def test_sdo_expedited_download_and_responses():
    assert wire.sdo_download_request(1, 0x1017, 0, 2, 100)["data"] == [0x2B, 0x17, 0x10, 0x00, 100, 0, 0, 0]
    assert wire.sdo_download_request(1, 0x6085, 0, 4, 10000000)["data"] == \
        [0x23, 0x85, 0x60, 0x00, 0x80, 0x96, 0x98, 0x00]
    assert wire.sdo_download_request(1, 0x6060, 0, 1, 8)["data"] == [0x2F, 0x60, 0x60, 0x00, 8, 0, 0, 0]
    assert wire.sdo_download_response(1, 0x1017, 0)["data"] == [0x60, 0x17, 0x10, 0x00, 0, 0, 0, 0]
    assert wire.sdo_upload_response(1, 0x1000, 0, 4, 0x00020192)["data"] == \
        [0x43, 0x00, 0x10, 0x00, 0x92, 0x01, 0x02, 0x00]
    assert wire.sdo_upload_response(1, 0x1017, 0, 2, 100)["cob_id"] == 0x581


def test_sync_and_emcy_ids_come_from_the_eds():
    assert (wire.sync(EDS)["cob_id"], wire.sync(EDS)["dlc"]) == (0x80, 0)
    f = wire.emcy(EDS, 6, 0x8130, 0x11, [1, 2, 3, 4, 5])
    assert (f["cob_id"], f["data"]) == (0x86, [0x30, 0x81, 0x11, 1, 2, 3, 4, 5])


def test_pdo_layouts_from_the_eds():
    pdos = {p["label"]: p for p in EDS.pdos()}
    assert list(pdos) == ["RPDO1", "RPDO2", "RPDO3", "TPDO1", "TPDO2", "TPDO3"]
    names = {k: [f["name"] for f in p["fields"]] for k, p in pdos.items()}
    assert names["RPDO1"] == ["target_position", "command_seq", "exec_tag"]
    assert names["TPDO1"] == ["statusword", "position_actual_value", "sample_seq"]
    assert [p["length"] for p in pdos.values()] == [8, 8, 2, 8, 8, 8]
    assert [wire.pdo_cob_id(p, 6) for p in pdos.values()] == [0x206, 0x306, 0x406, 0x186, 0x286, 0x386]


def test_pdo_encoding_is_little_endian_and_signed():
    rpdo1 = next(EDS.pdos())
    f = wire.pdo_frame(rpdo1, 1, {"target_position": -2, "command_seq": 0x0102, "exec_tag": 0xFFFF}, "x")
    assert (f["cob_id"], f["data"]) == (0x201, [0xFE, 0xFF, 0xFF, 0xFF, 0x02, 0x01, 0xFF, 0xFF])


def test_encoder_rejects_out_of_range_value():
    with pytest.raises(ValueError):
        wire.le(0x10000, 16)
    with pytest.raises(ValueError):
        wire.le(-(1 << 31) - 1, 32)


def test_golden_files_match_a_fresh_generation():
    fresh = make_golden.generate()
    assert sorted(fresh) == ["emcy.json", "heartbeat.json", "nmt.json", "pdo.json", "sdo.json", "sync.json"]
    for name, text in fresh.items():
        assert (make_golden.GOLDEN_DIR / name).read_text(encoding="utf-8") == text, name


def test_golden_generation_is_deterministic():
    assert make_golden.generate() == make_golden.generate()


def _load(name):
    return json.loads((make_golden.GOLDEN_DIR / f"{name}.json").read_text(encoding="utf-8"))


@pytest.mark.parametrize("family", ["nmt", "heartbeat", "sync", "emcy", "sdo", "pdo"])
def test_golden_entries_are_well_formed(family):
    entries = _load(family)
    assert entries
    names = [e["name"] for e in entries]
    assert len(names) == len(set(names))
    for e in entries:
        assert set(e) == {"name", "node", "cob_id", "dlc", "data", "fields"}
        assert e["dlc"] == len(e["data"]) <= 8
        assert all(0 <= b <= 255 for b in e["data"])
        assert 0 <= e["cob_id"] <= 0x7FF


def test_golden_pdo_edges_and_nodes():
    entries = _load("pdo")
    for p in EDS.pdos():
        mine = [e for e in entries if e["name"].startswith(p["label"].lower() + "_")]
        assert {e["node"] for e in mine} == {1, 6, 127}
        assert all(e["dlc"] == p["length"] for e in mine)
        for f in p["fields"]:
            lo, hi = wire.field_range(f)
            seen = {e["fields"][f["name"]] for e in mine}
            assert {lo, hi, 0} <= seen
    values = {k: {e["fields"][k] for e in entries if k in e["fields"]}
              for k in ("command_seq", "exec_tag", "valid_until_us", "target_position")}
    assert {0, 0xFFFF} <= values["command_seq"] and {1, 0xFFFF} <= values["exec_tag"]
    assert {0, 0xFFFFFFFF} <= values["valid_until_us"]
    assert {-(1 << 31), (1 << 31) - 1} <= values["target_position"]


def test_golden_pdo_data_decodes_back_to_fields():
    for e in _load("pdo"):
        pdo = next(p for p in EDS.pdos() if p["label"].lower() == e["name"].split("_")[0])
        for f in pdo["fields"]:
            raw = bytes(e["data"][f["offset"]:f["offset"] + f["bits"] // 8])
            assert int.from_bytes(raw, "little", signed=f["signed"]) == e["fields"][f["name"]]
