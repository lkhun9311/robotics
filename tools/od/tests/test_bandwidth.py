# SPDX-License-Identifier: Apache-2.0
"""Bandwidth of the wire contract: one frame recomputed by hand, and the occupancy bound."""
import pathlib

import bandwidth

EDS = pathlib.Path(__file__).resolve().parents[3] / "config" / "od" / "vdrive_arm.eds"


def test_eight_byte_frame_by_hand():
    # 47 overhead + 64 data bits = 111; stuffing bound floor((34 + 64 - 1) / 4) = floor(97 / 4) = 24.
    assert bandwidth.frame_bits(8) == 135
    assert bandwidth.frame_bits(0) == 47 + 8  # floor(33 / 4) = 8


def test_cycle_load_from_the_eds():
    r = bandwidth.compute(EDS)
    assert r["cyclic_frames_per_cycle"] == 31  # 1 SYNC + 6 * (2 RPDO + 3 TPDO)
    assert r["event_driven_not_counted"] == ["RPDO3"]
    assert r["conservative"]["bits_per_cycle"] == 31 * 135 == 4185
    assert r["conservative"]["occupancy_percent"] == 41.85
    assert r["conservative"]["occupancy_percent"] < 50
    assert r["actual_dlc"]["bits_per_cycle"] <= r["conservative"]["bits_per_cycle"]
    assert r["headroom"]["occupancy_percent_with_heartbeat_and_sdo"] < 50


def test_priority_order_starts_with_sync_then_by_cob_id():
    order = bandwidth.compute(EDS)["priority_order"]
    assert order[0].startswith("0x080 SYNC")
    ids = [int(s.split()[0], 16) for s in order if "event-driven" not in s]
    assert ids == sorted(ids)
