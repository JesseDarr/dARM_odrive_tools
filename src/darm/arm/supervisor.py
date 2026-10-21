"""Decides whether the arm is healthy. Reads telemetry; never sends anything.

Every check is a small function that names what is wrong in words a person can
act on. The control loop turns any finding into a frozen arm.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping, Sequence
from typing import Any

from darm.arm.drives import Drive
from darm.config import JointSpec, SafetySpec
from darm.odrive.node import ODriveNode, Stamped
from darm.odrive.protocol import AxisState, describe_errors


class Supervisor:
    def __init__(
        self,
        safety: SafetySpec,
        drives: Sequence[Drive],
        nodes: Mapping[int, ODriveNode],
        current_limits: Mapping[int, float],
    ) -> None:
        self._safety = safety
        self._drives = drives
        self._current_limits = current_limits
        self._nodes = [nodes[node_id] for drive in drives for node_id in drive.nodes]
        self._nodes_by_id = nodes
        self._first_seen: dict[str, float] = {}

    def check(self, now: float, expect_closed_loop: bool, commanded: Mapping[str, float] | None = None) -> list[str]:
        """Everything wrong right now. Joint checks only run once every node reports fresh data."""
        problems = [p for node in self._nodes for p in self._check_node(node, now, expect_closed_loop)]
        if problems:
            return problems
        return [p for drive in self._drives for p in self._check_drive(drive, now, commanded)]

    # ------------------------------------------------------------ per node

    def _check_node(self, node: ODriveNode, now: float, expect_closed_loop: bool) -> Iterator[str]:
        safety = self._safety
        name = f"node {node.node_id}"

        if node.heartbeat is None or node.heartbeat.age(now) > safety.heartbeat_timeout_s:
            yield f"{name}: silent (no heartbeat{_age(node.heartbeat, now)})"
            return
        if node.encoder is None or node.encoder.age(now) > safety.encoder_timeout_s:
            yield f"{name}: position feedback stale{_age(node.encoder, now)}"
        if node.axis_error:
            yield f"{name}: {describe_errors(node.axis_error)}"
        if expect_closed_loop and node.state is not AxisState.CLOSED_LOOP_CONTROL:
            yield f"{name}: not holding position (state {node.state.name})"
        if node.temperature and node.temperature.value.fet > safety.max_fet_temp_c:
            yield f"{name}: FETs at {node.temperature.value.fet:.0f} C (max {safety.max_fet_temp_c:.0f})"
        limit = self._current_limits.get(node.node_id)
        if node.iq and limit:
            amps = abs(node.iq.value.measured)
            if self._persists(f"{name}:strain", amps > safety.max_current_fraction * limit, now):
                yield f"{name}: straining at {amps:.0f} A of {limit:.0f} A (blocked or overloaded?)"
        if node.bus_power:
            volts = node.bus_power.value.voltage
            out_of_range = not safety.min_bus_voltage <= volts <= safety.max_bus_voltage
            if self._persists(f"{name}:voltage", out_of_range, now):
                allowed = f"{safety.min_bus_voltage:.0f}-{safety.max_bus_voltage:.0f}"
                yield f"{name}: bus at {volts:.1f} V (allowed {allowed})"

    # ------------------------------------------------------------ per joint

    def _check_drive(self, drive: Drive, now: float, commanded: Mapping[str, float] | None) -> Iterator[str]:
        nodes = [self._nodes_by_id[node_id] for node_id in drive.nodes]
        motor_turns = [node.encoder.value.pos for node in nodes if node.encoder]
        measured = drive.to_joint(motor_turns)
        settled = all(node.heartbeat and node.heartbeat.value.trajectory_done for node in nodes)

        for joint, value in zip(drive.joints, measured, strict=True):
            yield from self._check_limits(joint, value)
            if commanded is not None and joint.name in commanded:
                yield from self._check_tracking(joint, value, commanded[joint.name], settled, now)

    def _check_limits(self, joint: JointSpec, value: float) -> Iterator[str]:
        if not joint.min - joint.tolerance <= value <= joint.max + joint.tolerance:
            yield f"{joint.name}: at {value:.1f} {joint.unit}, outside {joint.min:g}..{joint.max:g}"

    def _check_tracking(
        self, joint: JointSpec, value: float, target: float, settled: bool, now: float
    ) -> Iterator[str]:
        """A trajectory that finished without arriving means something is in the way."""
        error = value - target
        if self._persists(f"{joint.name}:tracking", settled and abs(error) > joint.tolerance, now):
            yield f"{joint.name}: stuck {abs(error):.1f} {joint.unit} from target (blocked or overloaded?)"

    def _persists(self, key: str, condition: bool, now: float) -> bool:
        """True once condition has held continuously for safety.transient_s. Filters out spikes."""
        if not condition:
            self._first_seen.pop(key, None)
            return False
        return now - self._first_seen.setdefault(key, now) >= self._safety.transient_s


def _age(stamped: Stamped[Any] | None, now: float) -> str:
    return f" for {stamped.age(now) * 1000:.0f} ms" if stamped is not None else ""
