"""Supervisor checks against hand-built telemetry: every way the arm can be unhealthy, one at a time."""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from darm.arm import drives
from darm.arm.supervisor import Supervisor
from darm.config import RobotConfig
from darm.odrive.node import Stamped
from darm.odrive.protocol import (
    AxisState,
    BusVoltageCurrent,
    EncoderEstimates,
    Heartbeat,
    Iq,
    ODriveError,
    ProcedureResult,
    Temperature,
)

NOW = 1000.0


@dataclass
class FakeNode:
    """The slice of ODriveNode the supervisor reads."""

    node_id: int
    heartbeat: Stamped[Heartbeat] | None
    encoder: Stamped[EncoderEstimates] | None
    temperature: Stamped[Temperature] | None = None
    bus_power: Stamped[BusVoltageCurrent] | None = None
    iq: Stamped[Iq] | None = None

    @property
    def state(self) -> AxisState:
        return self.heartbeat.value.axis_state if self.heartbeat else AxisState.UNDEFINED

    @property
    def axis_error(self) -> int:
        return self.heartbeat.value.axis_error if self.heartbeat else 0


def healthy(node_id: int, pos: float = 0.0) -> FakeNode:
    return FakeNode(
        node_id,
        heartbeat=Stamped(Heartbeat(0, AxisState.CLOSED_LOOP_CONTROL, ProcedureResult.BUSY, True), NOW),
        encoder=Stamped(EncoderEstimates(pos, 0.0), NOW),
        temperature=Stamped(Temperature(40.0, float("nan")), NOW),
        bus_power=Stamped(BusVoltageCurrent(48.0, 1.0), NOW),
        iq=Stamped(Iq(1.0, 1.0), NOW),
    )


class Harness:
    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.drives = [drives.build(spec) for spec in config.drives]
        self.nodes = {node_id: healthy(node_id) for node_id in config.node_ids}
        limits = {n: lim for n in config.node_ids if (lim := config.current_limit(n)) is not None}
        self.supervisor = Supervisor(config.safety, self.drives, self.nodes, limits)  # type: ignore[arg-type]

    def check(self, now: float = NOW, commanded: dict[str, float] | None = None) -> list[str]:
        return self.supervisor.check(now, expect_closed_loop=True, commanded=commanded)

    def later(self, seconds: float, commanded: dict[str, float] | None = None) -> list[str]:
        """Check again `seconds` later, with every node still reporting the same values on time."""
        now = NOW + seconds
        for node in self.nodes.values():
            for name in ("heartbeat", "encoder", "temperature", "bus_power", "iq"):
                stamped = getattr(node, name)
                setattr(node, name, Stamped(stamped.value, now))
        return self.check(now, commanded)


@pytest.fixture
def h(config: RobotConfig) -> Harness:
    return Harness(config)


def test_a_healthy_arm_has_nothing_to_report(h: Harness) -> None:
    assert h.check(commanded=dict.fromkeys(h.config.joints, 0.0)) == []


def test_a_silent_node_is_reported_by_how_long_it_has_been_silent(h: Harness) -> None:
    assert sorted(h.check(now=NOW + 0.3)) == sorted(f"node {n}: silent (no heartbeat for 300 ms)" for n in h.nodes)


def test_stale_position_feedback_is_reported(h: Harness) -> None:
    h.nodes[4].encoder = Stamped(EncoderEstimates(0.0, 0.0), NOW - 0.2)
    assert h.check() == ["node 4: position feedback stale for 200 ms"]


def test_odrive_errors_are_reported_by_name(h: Harness) -> None:
    beat = Heartbeat(ODriveError.DRV_FAULT, AxisState.IDLE, ProcedureResult.DISARMED, True)
    h.nodes[2].heartbeat = Stamped(beat, NOW)
    assert h.check() == ["node 2: DRV_FAULT", "node 2: not holding position (state IDLE)"]


def test_hot_fets_are_reported(h: Harness) -> None:
    h.nodes[0].temperature = Stamped(Temperature(95.0, float("nan")), NOW)
    assert h.check() == ["node 0: FETs at 95 C (max 90)"]


def test_bus_voltage_must_stay_out_of_range_before_it_counts(h: Harness) -> None:
    h.nodes[0].bus_power = Stamped(BusVoltageCurrent(39.0, 1.0), NOW)
    assert h.check() == []
    assert h.later(0.6) == ["node 0: bus at 39.0 V (allowed 42-52)"]


def test_a_spike_that_clears_resets_the_timer(h: Harness) -> None:
    h.nodes[0].bus_power = Stamped(BusVoltageCurrent(39.0, 1.0), NOW)
    h.check()
    h.nodes[0].bus_power = Stamped(BusVoltageCurrent(48.0, 1.0), NOW)
    h.later(0.3)
    h.nodes[0].bus_power = Stamped(BusVoltageCurrent(39.0, 1.0), NOW)
    assert h.later(0.6) == []


def test_an_8308_held_near_its_current_limit_is_straining(h: Harness) -> None:
    h.nodes[3].iq = Stamped(Iq(38.0, -38.0), NOW)
    assert h.check() == []
    assert h.later(0.5) == ["node 3: straining at 38 A of 40 A (blocked or overloaded?)"]


def test_gimbal_motors_are_not_strain_checked(h: Harness) -> None:
    h.nodes[7].iq = Stamped(Iq(3.0, 3.0), NOW)
    h.check()
    assert h.later(1.0) == []


def test_a_joint_outside_its_limits_is_reported_in_joint_units(h: Harness) -> None:
    h.nodes[4].encoder = Stamped(EncoderEstimates(9.0 * 10 / 360, 0.0), NOW)  # elbow +10 deg, max is 0
    assert h.check() == ["elbow: at 10.0 deg, outside -236..0"]


def test_ganged_motors_that_disagree_point_at_a_mechanical_failure(h: Harness) -> None:
    h.nodes[2].encoder = Stamped(EncoderEstimates(0.25, 0.0), NOW)  # Shoulder motor B alone moved 10 deg.
    assert h.check() == ["shoulder: ganged motors disagree by 10.0 deg (slipped gear or broken part?)"]


def test_a_settled_joint_that_never_arrived_is_blocked(h: Harness) -> None:
    commanded = dict.fromkeys(h.config.joints, 0.0) | {"upper_arm": 20.0}
    assert h.check(commanded=commanded) == []
    assert h.later(0.5, commanded) == ["upper_arm: stuck 20.0 deg from target (blocked or overloaded?)"]
