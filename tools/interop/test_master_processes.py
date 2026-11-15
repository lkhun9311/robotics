# SPDX-License-Identifier: Apache-2.0
"""Our master as a process (canopen_master_cli, E3c Task 6) on vcan.

The CLI runs the canopen backend on its socket I/O worker. Every test first asserts its
preconditions (the steps before the one under test completed, the stream ran, the traces are
complete); a missing one fails with INCONCLUSIVE. These assert protocol facts, ordering and broad
deadlines; latencies are printed as measurements.
"""
import os
import pathlib

from conftest import HOST_IF, NODES, RELEASE, Proc, inconclusive

CLI_BIN = pathlib.Path(os.environ.get(
    "CANOPEN_MASTER_CLI_BIN",
    RELEASE / "canopen_master_cli" / "lib" / "canopen_master_cli" / "canopen_master_cli"))

QS_TIMEOUT, QS_EXPIRY = 1, 4  # drive_core QuickStopReason in the bank's `drive` records.
STOP_AFTER_STREAM_S = 0.160
MIN_DISPATCHED = 150
RPDO1, RPDO2, RPDO3 = 0x200, 0x300, 0x400


def start_cli(procs, scenario, *extra):
    assert CLI_BIN.exists(), f"NOT_RUN: {CLI_BIN} not built"
    p = Proc("canopen_master_cli", [CLI_BIN, "--if", HOST_IF, "--nodes", "1-6",
                                    "--scenario", scenario, *extra])
    procs.append(p)
    p.wait_for(lambda e: e.get("event") == "start", 2, "start line")
    return p


def run_to_end(cli, timeout):
    """Waits for the scenario's own end; returns (exit status, scenario_done or failed event)."""
    end = cli.wait_for(lambda e: e.get("event") in ("scenario_done", "scenario_failed"), timeout,
                       "end of the scenario")
    rc = cli.wait_exit(timeout=5)
    exits = cli.of("exit")
    if not exits:
        inconclusive(f"the CLI wrote no exit record; stderr: {cli.stderr[-500:]!r}")
    return rc, end


def step_done(cli, line):
    """The cmd_done of the scenario step named `line`, or None."""
    cmds = [e for e in cli.of("cmd") if e["line"] == line]
    if not cmds:
        return None
    done = cli.of("cmd_done", n=cmds[0]["n"])
    return done[0] if done else None


def test_master_drives_vdrive_bank(start_bank, proxy, procs):
    """Activate, arm, stream 2 s through the proxy: every seq the CLI recorded as dispatched for an
    approved command is actuated on all six drives, at least MIN_DISPATCHED of them, with no
    expiry or timeout stop while streaming; then the stream stops and every drive stops on its
    own (expiry or command timeout) within STOP_AFTER_STREAM_S."""
    bank = start_bank()
    bank.trace.wait_for(lambda r: r.get("event") == "start", 2, "bank start record")
    cli = start_cli(procs, "stream", "--duration-ms", "2000")
    rc, end = run_to_end(cli, 25)
    for step in ("configure", "activate", "arm"):
        done = step_done(cli, step)
        if done is None or not done["ok"]:
            inconclusive(f"step {step} did not complete: {done}; {end}")
    assert rc == 0 and end["event"] == "scenario_done", (rc, end, cli.stderr[-500:])
    stream_end = cli.of("stream_end")
    if len(stream_end) != 1:
        inconclusive(f"no single stream_end: {stream_end}")
    t_stream_end = stream_end[0]["t"]
    t_armed = step_done(cli, "arm")["t"]

    dispatched = [e["seq"] for e in cli.of("dispatch") if e["approved"]]
    holds = [e["seq"] for e in cli.of("dispatch") if not e["approved"]]
    print(f"dispatched {len(dispatched)} approved seqs (+ {len(holds)} arm holds); "
          f"superseded {end['superseded']}, expired before dispatch "
          f"{end['expired_before_dispatch']}, writes {end['writes']}, sent {end['sent']}")
    assert len(dispatched) >= MIN_DISPATCHED, f"only {len(dispatched)} dispatched seqs"
    assert len(set(dispatched)) == len(dispatched), "a dispatched seq was reported twice"
    # Superseded submissions are counted separately and never dispatched: every approved
    # submission is dispatched, superseded or expired, exactly once.
    assert end["sent"] == len(dispatched) + end["superseded"] + end["expired_before_dispatch"], end
    assert end["expired_before_dispatch"] == 0, end

    bank.stop()
    drive = [r for r in bank.trace.records if r.get("event") == "drive"]
    for node in NODES:
        actuated = {r["seq"] for r in drive if r["node"] == node and r["kind"] == "actuated"
                    and r["t_ns"] / 1e9 > t_armed - 0.1}
        missing = [s for s in dispatched if s not in actuated]
        assert not missing, f"node {node}: dispatched seqs never actuated: {missing[:10]}"
    early = [r for r in drive if r["kind"] == "quick_stop" and r["t_ns"] / 1e9 < t_stream_end]
    assert not early, f"a drive stopped while the stream ran: {early[:3]}"
    lags = []
    for node in NODES:
        stops = [r for r in drive if r["node"] == node and r["kind"] == "quick_stop"]
        assert stops, f"node {node} never stopped after the stream ended"
        first = stops[0]
        assert first["reason"] in (QS_TIMEOUT, QS_EXPIRY), f"node {node}: {first}"
        lag = first["t_ns"] / 1e9 - t_stream_end
        lags.append(round(lag * 1000, 1))
        assert 0 <= lag <= STOP_AFTER_STREAM_S, f"node {node} stopped {lag:.3f} s after the stream"
    print(f"stop after the stream ended, ms per node: {lags}")
    exit_record = cli.of("exit")[0]
    assert exit_record["event_overflows"] == 0, exit_record
    assert exit_record["tx_eagain"] == exit_record["tx_enobufs"] == 0, exit_record
    print(f"worker counters: {exit_record}")


def test_probe_slave_fails_activation(start_probe, capture, procs):
    """Lely BasicSlaves serving our EDS pass the identity probe of configure but are no CiA 402
    drive: they never report Switched On, so activation fails, and the master never sends
    Enable Operation (0x0F) nor a command PDO (RPDO1/RPDO2)."""
    cap = capture(HOST_IF)
    start_probe()
    cli = start_cli(procs, "activate_hal")
    rc, end = run_to_end(cli, 20)
    frames = cap.stop()
    configure = step_done(cli, "configure")
    if configure is None or not configure["ok"]:
        inconclusive(f"configure did not pass the probe: {configure}; {end}")
    activate = step_done(cli, "activate")
    if activate is None:
        inconclusive(f"activate never ended: {end}")
    assert not activate["ok"], activate
    assert rc == 1 and end["event"] == "scenario_failed", (rc, end)
    print(f"activation failed: {activate.get('error')}")
    controlwords = [f for f in frames if RPDO3 < f[1] <= RPDO3 + 6]
    if not controlwords:
        inconclusive("the master sent no controlword at all: activation was not attempted")
    enables = [f for f in frames if RPDO3 < f[1] <= RPDO3 + 6 and f[2][:2] == b"\x0f\x00"]
    assert not enables, f"Enable Operation sent: {enables[:3]}"
    commands = [f for f in frames
                if RPDO1 < f[1] <= RPDO1 + 6 or RPDO2 < f[1] <= RPDO2 + 6]
    assert not commands, f"command PDOs sent: {commands[:3]}"
    print(f"controlwords sent: {sorted({f[2][:2].hex() for f in controlwords})}")


def test_console_ends_at_stdin_eof_after_its_commands(start_bank, proxy, procs):
    """Without --duration-ms, closing stdin ends the console like `quit`: after the commands
    before it are done (their sdo_* and cmd_done lines first), with scenario_done and exit 0."""
    bank = start_bank()
    bank.trace.wait_for(lambda r: r.get("event") == "start", 2, "bank start record")
    assert CLI_BIN.exists(), f"NOT_RUN: {CLI_BIN} not built"
    cli = Proc("canopen_master_cli", [CLI_BIN, "--if", HOST_IF, "--nodes", "3",
                                      "--scenario", "console"], stdin=True)
    procs.append(cli)
    cli.wait_for(lambda e: e.get("event") == "start", 2, "start line")
    cli.send("read 3 0x1000 0 u32")
    cli.proc.stdin.close()
    rc, end = run_to_end(cli, 5)
    assert rc == 0 and end["event"] == "scenario_done", (rc, end, cli.stderr[-500:])
    order = [e["event"] for e in cli.events
             if e.get("event") in ("sdo_ok", "cmd_done", "scenario_done", "scenario_failed")]
    assert order == ["sdo_ok", "cmd_done", "scenario_done"], cli.events[-10:]
    assert [e["value"] for e in cli.of("sdo_ok", node=3, index=0x1000)] == [0x00020192]
