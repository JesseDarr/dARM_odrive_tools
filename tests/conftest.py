"""Shared fixtures: the real darm.toml, and a simulated arm on a private virtual CAN channel.

Two configs. ROBOT is darm.toml exactly; tests of the supervisor's timeouts run it
against fake clocks, where every deadline is exact. LIVE is ROBOT with timeouts wide
enough for a desktop OS, for tests that run threads in real time: under load,
Windows and the GIL can hold a Python thread for longer than the 100 ms the
supervisor allows a position frame, and the arm then freezes, correctly, for a
reason that has nothing to do with what the test is checking.
"""

import time
import uuid
from collections.abc import Callable, Iterator
from dataclasses import dataclass, replace
from pathlib import Path

import pytest

from darm import config as cfg
from darm.arm.arm import Arm
from darm.arm.journal import Journal
from darm.odrive.bus import CanBus
from darm.sim import SimNetwork

REPO = Path(__file__).resolve().parents[1]
ROBOT = cfg.load(REPO / "darm.toml")
LIVE = replace(ROBOT, safety=replace(ROBOT.safety, heartbeat_timeout_s=1.0, encoder_timeout_s=0.5))


def node_of(joint: str) -> int:
    """The node driving a joint (the first of the two, for a differential). Tests never hard-code node ids."""
    return next(drive.nodes[0] for drive in ROBOT.drives if joint in {j.name for j in drive.joints})


def motor_turns(joint: str, value: float) -> float:
    """Where a direct-drive joint's motor sits, in turns, when the joint is at value."""
    return ROBOT.drive_for_node(node_of(joint)).turns_per_unit * value


@pytest.fixture(scope="session")
def config() -> cfg.RobotConfig:
    return ROBOT


@pytest.fixture(scope="session")
def live_config() -> cfg.RobotConfig:
    return LIVE


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
def make_rig(live_config: cfg.RobotConfig, tmp_path: Path) -> Iterator[Callable[..., Rig]]:
    """Build a rig. prepare=fn(sim) changes the simulated ODrives before the arm discovers them.

    prepare runs without the simulator lock held: place() takes it itself, and the
    single-attribute changes the tests make are atomic anyway.
    """
    opened: list[Rig] = []

    def build(prepare: Callable[[SimNetwork], None] | None = None, node_ids: tuple[int, ...] | None = None) -> Rig:
        channel = f"test-{uuid.uuid4().hex}"
        sim = SimNetwork(live_config, channel, node_ids)
        if prepare is not None:
            prepare(sim)
        bus = CanBus(channel, "virtual", live_config.node_ids)
        journal = Journal(tmp_path / "journal.json")
        rig = Rig(sim, bus, Arm(live_config, bus, journal), journal)
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
