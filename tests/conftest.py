"""Shared fixtures: the real darm.toml, and a simulated arm on a private virtual CAN channel."""

from __future__ import annotations

import time
import uuid
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from pathlib import Path

import pytest

from darm import config as cfg
from darm.arm.arm import Arm
from darm.arm.journal import Journal
from darm.odrive.bus import CanBus
from darm.sim import SimNetwork

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def config() -> cfg.RobotConfig:
    return cfg.load(REPO / "darm.toml")


@pytest.fixture
def config_text() -> str:
    return (REPO / "darm.toml").read_text(encoding="utf-8")


@dataclass
class Rig:
    """A simulated arm, the host-side Arm watching it, and the bus between them."""

    sim: SimNetwork
    bus: CanBus
    arm: Arm
    journal: Journal


@pytest.fixture
def make_rig(config: cfg.RobotConfig, tmp_path: Path) -> Iterator[Callable[..., Rig]]:
    """Build a rig. prepare=fn(sim) changes the simulated ODrives before the arm discovers them.

    prepare runs without the simulator lock held: place() takes it itself, and the
    single-attribute changes the tests make are atomic anyway.
    """
    opened: list[Rig] = []

    def build(prepare: Callable[[SimNetwork], None] | None = None, node_ids: tuple[int, ...] | None = None) -> Rig:
        channel = f"test-{uuid.uuid4().hex}"
        sim = SimNetwork(config, channel, node_ids)
        if prepare is not None:
            prepare(sim)
        bus = CanBus(channel, "virtual", config.node_ids)
        journal = Journal(tmp_path / "journal.json")
        rig = Rig(sim, bus, Arm(config, bus, journal), journal)
        opened.append(rig)
        rig.arm.discover()
        return rig

    yield build
    for rig in opened:
        rig.arm.close()
        rig.bus.close()
        rig.sim.close()


def eventually(condition: Callable[[], bool], timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.01)
    return condition()
