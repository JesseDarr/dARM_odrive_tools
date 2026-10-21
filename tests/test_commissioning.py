"""The commissioning tools, run for real against a simulated arm the test can reach into."""

from __future__ import annotations

import argparse
import dataclasses
import json
import threading
import uuid
from collections.abc import Callable, Iterable, Iterator
from contextlib import contextmanager
from pathlib import Path

import pytest

from darm import cli
from darm.apps import address, backup, calibrate, configure, errors, exercise, home
from darm.cli import Context
from darm.config import ConfigError, RobotConfig
from darm.odrive.bus import CanBus
from darm.odrive.endpoints import EndpointTable
from darm.odrive.protocol import AxisState, ODriveError, ProcedureResult
from darm.sim import SimNetwork, place
from tests.conftest import REPO, node_of

TOOLS = (address, backup, calibrate, configure, errors, exercise, home)


@dataclasses.dataclass(frozen=True)
class SimContext(Context):
    """A Context whose bus is the test's own simulated network, so the test can change the ODrives."""

    channel: str = ""

    @contextmanager
    def bus(self, node_ids: Iterable[int] | None = None) -> Iterator[CanBus]:
        with CanBus(self.channel, "virtual", node_ids) as bus:
            yield bus


Station = Callable[..., tuple[SimNetwork, SimContext]]


@pytest.fixture
def station(config: RobotConfig, tmp_path: Path) -> Iterator[Station]:
    """Build a simulated arm and a Context wired to it."""
    networks: list[SimNetwork] = []

    def build(node_ids: tuple[int, ...] | None = None, freshly_flashed: bool = False) -> tuple[SimNetwork, SimContext]:
        channel = f"test-{uuid.uuid4().hex}"
        sim = SimNetwork(config, channel, node_ids, freshly_flashed)
        networks.append(sim)
        return sim, SimContext(config, True, tmp_path / "state", channel)

    yield build
    for sim in networks:
        sim.close()


@pytest.fixture(scope="module")
def table(config: RobotConfig) -> EndpointTable:
    return EndpointTable.for_firmware(config.firmware)


def run_tool(ctx: Context, *argv: str) -> int:
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers()
    for tool in TOOLS:
        tool.register(commands)
    args = parser.parse_args(argv)
    return int(args.run(args, ctx))


def tamper(sim: SimNetwork, table: EndpointTable, node_id: int, path: str, value: float) -> None:
    endpoint = table[path]
    with sim.locked():
        sim.nodes[node_id].values[endpoint.id] = endpoint.encode(value)


def stored(sim: SimNetwork, table: EndpointTable, node_id: int, path: str) -> object:
    endpoint = table[path]
    return endpoint.decode(sim.nodes[node_id].values[endpoint.id])


# ------------------------------------------------------------------------------ configure


def test_configure_dry_run_reports_differences_and_writes_nothing(station: Station, table: EndpointTable, capsys):
    sim, ctx = station()
    tamper(sim, table, 3, "axis0.config.motor.pole_pairs", 21)

    assert run_tool(ctx, "configure", "--dry-run") == 0

    out = capsys.readouterr().out
    assert "axis0.config.motor.pole_pairs" in out
    assert "21 -> 20" in out
    assert stored(sim, table, 3, "axis0.config.motor.pole_pairs") == 21
    assert not sim.nodes[3].saved


def test_configure_writes_only_differences_then_saves(station: Station, table: EndpointTable):
    sim, ctx = station()
    tamper(sim, table, 3, "axis0.config.motor.pole_pairs", 21)
    tamper(sim, table, 3, "axis0.controller.config.vel_gain", 0.5)

    assert run_tool(ctx, "configure", "--yes") == 0

    assert stored(sim, table, 3, "axis0.config.motor.pole_pairs") == 20
    assert stored(sim, table, 3, "axis0.controller.config.vel_gain") == pytest.approx(0.55)
    assert sim.nodes[3].saved
    assert not sim.nodes[0].saved, "a node that was already right must not be rebooted"


def test_configure_refuses_while_any_node_is_holding(station: Station, table: EndpointTable):
    sim, ctx = station()
    tamper(sim, table, 3, "axis0.config.motor.pole_pairs", 21)
    with sim.locked():
        sim.nodes[0].state = AxisState.CLOSED_LOOP_CONTROL

    assert run_tool(ctx, "configure", "--yes") == 1
    assert stored(sim, table, 3, "axis0.config.motor.pole_pairs") == 21


def test_configure_refuses_a_baud_rate_that_would_strand_the_nodes(config: RobotConfig, tmp_path: Path):
    stranded = dataclasses.replace(config, common_settings={**config.common_settings, "can.config.baud_rate": 500_000})
    with pytest.raises(ConfigError, match="would vanish"):
        run_tool(Context(stranded, True, tmp_path), "configure", "--yes")


def test_configure_compares_as_the_odrive_stores_values():
    assert configure.same(0.10599999874830246, 0.106)  # float32 round trip
    assert configure.same(float("nan"), float("nan"))
    assert not configure.same(float("nan"), 0.0)
    assert configure.same(True, 1)
    assert not configure.same(20, 21)


# ------------------------------------------------------------------------------ calibrate


def test_calibrate_saves_a_successful_calibration(station: Station):
    sim, ctx = station()
    assert run_tool(ctx, "calibrate", "3", "--yes") == 0
    assert sim.nodes[3].saved


def test_calibrate_reports_a_failure_and_does_not_save_it(station: Station, caplog):
    sim, ctx = station()
    with sim.locked():
        sim.nodes[3].calibration_result = ProcedureResult.PHASE_RESISTANCE_OUT_OF_RANGE

    assert run_tool(ctx, "calibrate", "3", "--yes") == 1
    assert not sim.nodes[3].saved
    assert "PHASE_RESISTANCE_OUT_OF_RANGE" in caplog.text


def test_calibrate_refuses_a_node_with_errors(station: Station):
    sim, ctx = station()
    with sim.locked():
        sim.nodes[3].fault(ODriveError.DRV_FAULT)

    assert run_tool(ctx, "calibrate", "3", "--yes") == 1
    assert sim.nodes[3].procedure_result is ProcedureResult.SUCCESS, "calibration must not have started"


def test_calibrate_targets(config: RobotConfig):
    assert calibrate._targets([], "shoulder", config) == (node_of("shoulder"),)
    wrist = node_of("wrist_twist")
    assert calibrate._targets([], "wrist_twist", config) == (wrist, wrist + 1)
    assert calibrate._targets([4, 4, 6], None, config) == (4, 6)
    for nodes, joint in (([], None), ([3], "elbow"), ([], "tail"), ([9], None)):
        with pytest.raises(ConfigError):
            calibrate._targets(nodes, joint, config)


# ------------------------------------------------------------------------------ errors


def test_errors_reports_then_clears(station: Station, capsys):
    sim, ctx = station()
    with sim.locked():
        sim.nodes[4].fault(ODriveError.DRV_FAULT)

    assert run_tool(ctx, "errors") == 1
    assert "DRV_FAULT" in capsys.readouterr().out

    assert run_tool(ctx, "errors", "--clear") == 0
    assert sim.nodes[4].axis_error == 0
    assert sim.nodes[4].disarm_reason == 0


# ------------------------------------------------------------------------------ home


def test_home_zeroes_every_joint_and_marks_the_arm_docked(station: Station):
    sim, ctx = station()
    place(sim, {0: 0.4, 3: -0.2, 5: 0.3, 6: 0.1})

    assert run_tool(ctx, "home", "--yes") == 0
    assert all(node.pos == 0.0 for node in sim.nodes.values())
    assert ctx.journal().was_docked() is True


def test_home_refuses_while_the_arm_is_held(station: Station):
    sim, ctx = station()
    place(sim, {0: 0.4})
    with sim.locked():
        sim.nodes[0].state = AxisState.CLOSED_LOOP_CONTROL

    assert run_tool(ctx, "home", "--yes") == 1
    assert sim.nodes[0].pos == 0.4
    assert ctx.journal().was_docked() is None


# ------------------------------------------------------------------------------ backup


def test_backup_writes_every_readable_setting(station: Station, table: EndpointTable, tmp_path: Path):
    gripper = node_of("gripper")
    _, ctx = station(node_ids=(gripper,))  # The station keeps the network alive.
    out = tmp_path / "backup.json"

    assert run_tool(ctx, "backup", "--out", str(out)) == 0

    document = json.loads(out.read_text(encoding="utf-8"))
    assert document["firmware"] == ctx.config.firmware
    assert list(document["nodes"]) == [str(gripper)]
    settings = document["nodes"][str(gripper)]
    assert settings["axis0.config.motor.pole_pairs"] == 11
    assert "save_configuration" not in settings, "functions are not settings"
    assert len(settings) == sum(1 for e in table if backup._readable(e))


def test_backup_refuses_a_firmware_the_odrives_do_not_run(station: Station, tmp_path: Path):
    _, ctx = station(node_ids=(node_of("gripper"),))  # The station keeps the network alive.
    out = tmp_path / "backup.json"
    assert run_tool(ctx, "backup", "--firmware", "0.6.10", "--out", str(out)) == 1
    assert not out.exists()


# ------------------------------------------------------------------------------ exercise


def test_exercise_sweeps_the_joint_and_puts_it_back(station: Station, config: RobotConfig):
    sim, ctx = station()
    gripper = sim.nodes[node_of("gripper")]
    samples: list[float] = []
    running = threading.Event()
    running.set()

    def sample() -> None:
        while running.is_set():
            samples.append(gripper.pos)
            running.wait(0.005)

    sampler = threading.Thread(target=sample, daemon=True)
    sampler.start()
    try:
        assert run_tool(ctx, "exercise", "gripper", "--cycles", "1", "--yes") == 0
    finally:
        running.clear()
        sampler.join()

    turns_per_mm = config.drive_for_node(node_of("gripper")).turns_per_unit
    low, high = exercise.sweep_range(config.joints["gripper"], 0.9)
    assert max(samples) == pytest.approx(high * turns_per_mm, abs=0.01)
    assert min(samples) == pytest.approx(low * turns_per_mm, abs=0.01)
    assert gripper.pos == pytest.approx(0.0, abs=0.01)


def test_sweep_range_stays_clear_of_both_limits(config: RobotConfig):
    shoulder = config.joints["shoulder"]
    assert exercise.sweep_range(shoulder, 0.9) == pytest.approx((-190.0, -10.0))
    assert exercise.sweep_range(shoulder, 1.0) == pytest.approx((shoulder.min, shoulder.max))


# ------------------------------------------------------------------------------ through the real CLI


@pytest.mark.parametrize("argv", [["errors"], ["configure", "--dry-run"]])
def test_tools_run_end_to_end_through_the_cli(argv: list[str], tmp_path: Path):
    common = ["--config", str(REPO / "darm.toml"), "--sim", "--state-dir", str(tmp_path)]
    assert cli.main([*common, *argv]) == 0
