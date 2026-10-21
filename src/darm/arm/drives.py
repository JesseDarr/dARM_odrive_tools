"""Transmissions: joint units <-> motor turns.

Two mechanisms cover every joint on the arm:
  DirectDrive        one motor, one joint
  DifferentialDrive  two motors, two joints: turning together twists, turning apart bends

Both are linear, so the same mapping converts positions and velocities.
"""

from __future__ import annotations

import abc
from collections.abc import Sequence

from darm.config import DriveSpec, JointSpec


class Drive(abc.ABC):
    def __init__(self, spec: DriveSpec) -> None:
        self.spec = spec
        self.nodes = spec.nodes
        self.joints = spec.joints
        self._turns_per_unit = spec.turns_per_unit

    @abc.abstractmethod
    def to_motor(self, joint_values: Sequence[float]) -> tuple[float, ...]:
        """Joint values (in self.joints order) -> motor turns (in self.nodes order)."""

    @abc.abstractmethod
    def to_joint(self, motor_turns: Sequence[float]) -> tuple[float, ...]:
        """Motor turns (in self.nodes order) -> joint values (in self.joints order)."""


class DirectDrive(Drive):
    def to_motor(self, joint_values: Sequence[float]) -> tuple[float, ...]:
        (value,) = joint_values
        return (self._turns_per_unit * value,)

    def to_joint(self, motor_turns: Sequence[float]) -> tuple[float, ...]:
        (turns,) = motor_turns
        return (turns / self._turns_per_unit,)


class DifferentialDrive(Drive):
    """joints = (bend, twist). motor_a = k(twist + bend), motor_b = k(twist - bend)."""

    def to_motor(self, joint_values: Sequence[float]) -> tuple[float, ...]:
        bend, twist = joint_values
        k = self._turns_per_unit
        return (k * (twist + bend), k * (twist - bend))

    def to_joint(self, motor_turns: Sequence[float]) -> tuple[float, ...]:
        a, b = motor_turns
        k2 = 2.0 * self._turns_per_unit
        return ((a - b) / k2, (a + b) / k2)


def build(spec: DriveSpec) -> Drive:
    return DifferentialDrive(spec) if spec.kind == "differential" else DirectDrive(spec)


def joint_names(drives: Sequence[Drive]) -> list[str]:
    return [joint.name for drive in drives for joint in drive.joints]


def joint_specs(drives: Sequence[Drive]) -> dict[str, JointSpec]:
    return {joint.name: joint for drive in drives for joint in drive.joints}
