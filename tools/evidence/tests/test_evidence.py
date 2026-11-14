# SPDX-License-Identifier: Apache-2.0
"""The committed E3b evidence is what the committed scripts make from the committed inputs."""
import json

import pytest

import common
import fault_timeline
import frame_trace
import matrix_table
import pick_place_replay
import proxy_cost_histogram

MEDIA = common.MEDIA


def test_frame_trace_is_regenerated_from_the_m5_recording():
    text = frame_trace.build(common.load_recording("m5"), 21, "m5")
    assert text == (MEDIA / "e3b-frame-trace.txt").read_text(encoding="utf-8")


def test_frame_trace_refuses_a_seq_without_one_actuation():
    obs = common.load_recording("m5")
    obs["bank"] = [r for r in obs["bank"] if not (r.get("kind") == "actuated" and r["seq"] == 21)]
    with pytest.raises(SystemExit, match="0 actuation records"):
        frame_trace.build(obs, 21, "m5")


def stream_t0(obs):
    """The first stream RPDO1 on vcan_device, found here without the script's helpers."""
    tag = [e for e in obs["peer"] if e.get("event") == "scenario_done"][0]["tag"]
    rpdo1 = common.cob("RPDO1")
    return min(t for t, c, h in obs["device"]
               if c == rpdo1 and len(h) == 16 and
               int.from_bytes(bytes.fromhex(h)[6:8], "little") == tag)


def drive_marks_expected(obs):
    """The bank's node-3 drive records inside the plotted window, as (kind, x ms, label)."""
    t0 = stream_t0(obs)
    names = {0: "other_tag", 2: "not_valid", 3: "missing_half"}
    out = []
    for r in obs["bank"]:
        if r.get("event") != "drive" or r.get("node") != common.NODE:
            continue
        x = (r["t_ns"] / 1e9 - t0) * 1e3
        if not fault_timeline.T0 <= x <= fault_timeline.T1:
            continue
        if r["kind"] == "actuated":
            out.append(("actuated", round(x, 6), r["seq"]))
        elif r["kind"] == "dropped" and r["drop"] in names:
            out.append((names[r["drop"]], round(x, 6), r["count"]))
        elif r["kind"] == "quick_stop":
            out.append(("quick_stop", round(x, 6), fault_timeline.QS_NAMES[r["reason"]]))
    return sorted(out, key=str)


@pytest.mark.parametrize("row", ["f1", "f7"])
def test_timeline_plots_every_drive_record_of_the_window(row):
    data = fault_timeline.panel_data(row)
    plotted = sorted(((m["kind"], round(m["x"], 6), m["label"]) for m in data["marks"]
                      if m["lane"] == "drive"), key=str)
    assert plotted == drive_marks_expected(common.load_recording(row))


def test_timeline_f1_shows_the_timeout_stop_and_no_actuation():
    kinds = [m for m in fault_timeline.panel_data("f1")["marks"] if m["lane"] == "drive"]
    assert [m["label"] for m in kinds if m["kind"] == "quick_stop"] == ["timeout"]
    assert not [m for m in kinds if m["kind"] == "actuated"]
    stop = next(m["x"] for m in kinds if m["kind"] == "quick_stop")
    assert any(state == 0x07 and a >= stop for a, _, state in fault_timeline.panel_data("f1")
               ["states"])


def test_timeline_f7_late_rpdo2_are_the_held_ones_and_never_actuated():
    obs = common.load_recording("f7")
    t0 = stream_t0(obs)
    tag = common.stream_tag(obs)
    host = {}
    for t, s, _ in common.rpdo_of_tag(obs, "host", "RPDO2", tag):
        host.setdefault(s, t)
    held = set()
    for t, s, _ in common.rpdo_of_tag(obs, "device", "RPDO2", tag):
        if (t - host[s]) * 1e3 > 30 and fault_timeline.T0 <= (t - t0) * 1e3 <= fault_timeline.T1:
            held.add(s)
    data = fault_timeline.panel_data("f7")
    late = {m["label"] for m in data["marks"] if m["kind"] == "late"}
    actuated = {m["label"] for m in data["marks"] if m["kind"] == "actuated"}
    assert late == held and late
    assert not late & actuated
    assert len([m for m in data["marks"] if m["kind"] == "not_valid"]) == len(late)


def test_histogram_plots_the_measured_distribution():
    cost = json.loads((MEDIA / "e3b-proxy-cost.json").read_text())
    data = proxy_cost_histogram.plot_data(cost, 200)
    for start_us, n in data["bars"]:
        lo = round(start_us * 1000)
        assert n == sum(c for v, c in cost["dist"] if lo <= v < lo + 200)
    assert sum(n for _, n in data["bars"]) == cost["samples"] == cost["count"]
    assert data["markers"] == {"p50": cost["p50"] / 1000, "p99": cost["p99"] / 1000,
                               "max": cost["max"] / 1000}


@pytest.fixture(scope="module")
def kin():
    return pick_place_replay.Kinematics()


@pytest.fixture(scope="module")
def pick_place_rec():
    return pick_place_replay.load()


def test_pick_place_recording_is_the_current_demo_and_passes_the_sim_check_conditions(pick_place_rec, kin):
    """The committed recording is of today's waypoints and schedule, covers the goal at 100 Hz,
    and passes the same conditions as the simulation check."""
    from pick_place import DEMO_SCHEDULE, WAYPOINTS, failures

    from expected import JOINTS

    rec = pick_place_rec
    assert rec["joints"] == list(JOINTS)
    assert [tuple(x) for x in rec["schedule"]] == list(DEMO_SCHEDULE)
    assert {k: tuple(v) for k, v in rec["waypoints"].items()} == WAYPOINTS
    stamps = [row[0] for row in rec["samples"]]
    assert stamps[0] <= 0.01 and stamps[-1] >= rec["result_s"] + 1.0 >= DEMO_SCHEDULE[-1][2]
    # Every fifth 500 Hz message is kept, so one missing message stretches a step to 12 ms (the
    # committed recording has one); 20 ms bounds the hole a missing message can leave.
    assert all(0 < b - a <= 0.02 for a, b in zip(stamps, stamps[1:])), "gap or disorder in the 100 Hz samples"
    assert failures(*pick_place_replay.judged(rec, kin)) == []


def test_pick_place_height_plot_draws_mujocos_site_height_per_sample(pick_place_rec, kin):
    data = pick_place_replay.plot_data(pick_place_rec, kin)
    rows = pick_place_rec["samples"]
    assert data["t"] == [row[0] for row in rows]
    assert data["site_z"] == [kin.site(dict(zip(pick_place_rec["joints"], row[1:])))[0][2] for row in rows]
    # The carried interval: the lowest site sample of the first PICK and PLACE dwells, found here
    # from the plotted heights and the schedule.
    def lowest(name):
        _, arrive, leave = next(x for x in pick_place_rec["schedule"] if x[0] == name)
        return min((z, t) for t, z in zip(data["t"], data["site_z"]) if arrive <= t <= leave + 0.2)[1]
    assert data["carried"] == (lowest("PICK"), lowest("PLACE"))
    assert data["lowest_carry"] >= data["carry_min_z"]


def test_drawn_object_rests_rides_with_the_tool_only_while_carried_then_rests(pick_place_rec, kin):
    from expected import OBJECT_BELOW_SITE, OBJECT_RADIUS, TABLE_Z
    from pick_place import CONTACT_ALLOWANCE

    track = pick_place_replay.object_track(pick_place_rec, kin)
    attach, release = pick_place_replay.carried_interval(pick_place_rec, kin)
    before = {c for t, c, carried in track if t < attach}
    after = {c for t, c, carried in track if t > release}
    assert len(before) == 1 and len(after) == 1, "the resting object moved"
    assert next(iter(before))[2] == next(iter(after))[2] == TABLE_Z + OBJECT_RADIUS
    jumps = []
    for (t, centre, carried), row in zip(track, pick_place_rec["samples"]):
        assert carried == (attach <= t <= release)
        if carried:
            pos, axis = kin.site(dict(zip(pick_place_rec["joints"], row[1:])))
            assert centre == tuple(p + OBJECT_BELOW_SITE * a for p, a in zip(pos, axis))
        assert centre[2] - OBJECT_RADIUS >= TABLE_Z - CONTACT_ALLOWANCE, f"object into the table at {t} s"
        if t in (attach, release):
            rest = next(iter(before if t == attach else after))
            jumps.append(max(abs(a - b) for a, b in zip(centre, rest)))
    # Attach and release move the drawn object by at most this much (it sits on the table, the
    # tool stops a few millimetres above or below).
    assert len(jumps) == 2 and max(jumps) <= 0.015, jumps


def test_pick_place_videos_are_committed_and_the_gif_fits_the_readme():
    assert pick_place_replay.MP4.stat().st_size > 0
    assert 0 < pick_place_replay.GIF.stat().st_size <= 3 * 1024 * 1024
    assert (MEDIA / "e2-baseline.mp4").stat().st_size > 0


FRESH = [("e3b-fault-timeline.svg", "fault_timeline.py",
          lambda out: fault_timeline.draw(out)),
         ("e2-pick-place-height.svg", "pick_place_replay.py",
          lambda out: pick_place_replay.draw_svg(pick_place_replay.load(), out, pick_place_replay.Kinematics())),
         ("e3b-proxy-cost.svg", "proxy_cost_histogram.py",
          lambda out: proxy_cost_histogram.draw(
              json.loads((MEDIA / "e3b-proxy-cost.json").read_text()), out, 200))]


@pytest.mark.parametrize("name,script,draw", FRESH, ids=[f[0] for f in FRESH])
def test_committed_svg_is_fresh(tmp_path, name, script, draw):
    """Freshness, not data: the data are checked above. The bytes depend on matplotlib and the
    font file, which the Ubuntu archive may update; then the image is regenerated, not fixed."""
    regenerate = f"regenerate with python3 tools/evidence/{script} in the dev image and commit"
    recorded = common.recorded_toolchain(MEDIA / name)
    assert recorded == common.toolchain(), (
        f"toolchain changed ({recorded!r} in {name}, {common.toolchain()!r} here): {regenerate}")
    out = tmp_path / name
    draw(out)
    assert out.read_bytes() == (MEDIA / name).read_bytes(), (
        f"{name} differs from what its script draws from the committed input: {regenerate}")


def test_matrix_table_never_reports_a_missing_or_failed_row_as_pass(tmp_path):
    junit = tmp_path / "junit.xml"
    junit.write_text(
        '<testsuites><testsuite name="pytest" tests="2">'
        '<testcase classname="tools.interop.test_matrix" name="test_nmt_states_and_service_gating"/>'
        '<testcase classname="tools.interop.test_matrix" name="test_sdo_expedited_and_aborts">'
        '<failure message="M3 FAIL"/></testcase></testsuite></testsuites>')
    rows = {line.split("|")[1].strip(): line.split("|")[4].strip()
            for line in matrix_table.table(junit).splitlines()[2:]}
    assert rows["M1"] == "PASS"
    assert rows["M3"] == "FAIL"
    assert rows["M2"] == "MISSING" and rows["R1"] == "MISSING"
