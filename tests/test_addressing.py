"""Address-based discovery and node id assignment, against simulated ODrives fresh from a flash."""

import argparse
import uuid
from collections.abc import Callable, Iterator
from pathlib import Path

import pytest

from darm import cli
from darm.apps import address
from darm.config import RobotConfig
from darm.odrive.addressing import AddressBook, assign, enumerate_odrives, free_node_id
from darm.odrive.bus import CanBus
from darm.odrive.protocol import BROADCAST_NODE_ID, MAX_NODE_ID
from darm.sim import SimNetwork, simulated_serial
from tests.conftest import REPO, ROBOT, eventually
from tests.test_commissioning import SimContext


@pytest.fixture
def channel() -> str:
    return f"test-{uuid.uuid4().hex}"


@pytest.fixture
def make_sim(config: RobotConfig, channel: str) -> Iterator[Callable[..., SimNetwork]]:
    networks: list[SimNetwork] = []

    def build(node_ids: tuple[int, ...] | None = None, freshly_flashed: bool = False) -> SimNetwork:
        sim = SimNetwork(config, channel, node_ids, freshly_flashed)
        networks.append(sim)
        return sim

    yield build
    for sim in networks:
        sim.close()


def run_address(ctx: SimContext, *argv: str) -> int:
    parser = argparse.ArgumentParser()
    address.register(parser.add_subparsers())
    args = parser.parse_args(["address", *argv])
    return int(args.run(args, ctx))


def serial_text(node_id: int) -> str:
    return f"{simulated_serial(node_id):012X}"


def test_enumerate_finds_every_odrive_by_serial(make_sim, channel: str):
    make_sim()
    with CanBus(channel, "virtual") as bus:
        found = enumerate_odrives(bus, AddressBook(bus), quiet_s=0.3)
    assert {reply.serial_number: reply.node_id for reply in found.values()} == {
        simulated_serial(n): n for n in ROBOT.node_ids
    }


def test_enumerate_sees_unaddressed_odrives(make_sim, channel: str):
    make_sim(node_ids=(0, 1), freshly_flashed=True)
    with CanBus(channel, "virtual") as bus:
        found = enumerate_odrives(bus, AddressBook(bus), quiet_s=0.3)
    assert sorted(found) == [simulated_serial(0), simulated_serial(1)]
    assert all(reply.node_id == BROADCAST_NODE_ID for reply in found.values())


def test_assign_gives_unaddressed_odrives_their_ids_and_saves(make_sim, channel, config, tmp_path: Path):
    sim = make_sim(node_ids=(0, 3, 6), freshly_flashed=True)
    ctx = SimContext(config, True, tmp_path, channel)

    assert run_address(ctx, "--assign", *(f"{serial_text(n)}={n}" for n in (0, 3, 6))) == 0

    assert sorted(sim.nodes) == [0, 3, 6]
    assert all(node.serial_number == simulated_serial(n) for n, node in sim.nodes.items())
    assert all(node.saved for node in sim.nodes.values())


def test_assign_refuses_an_odrive_it_did_not_find(make_sim, channel, config, tmp_path: Path):
    sim = make_sim(node_ids=(0,), freshly_flashed=True)
    ctx = SimContext(config, True, tmp_path, channel)

    assert run_address(ctx, "--assign", "0123456789AB=0") == 1
    assert sim.nodes == {}


def test_assign_refuses_to_take_an_id_another_odrive_keeps(make_sim, channel, config, tmp_path: Path):
    sim = make_sim(node_ids=(0, 3))
    ctx = SimContext(config, True, tmp_path, channel)

    assert run_address(ctx, "--assign", f"{serial_text(3)}=0") == 1
    assert sim.nodes[0].serial_number == simulated_serial(0)
    assert sim.nodes[3].serial_number == simulated_serial(3)


def test_interactive_blinks_each_odrive_and_asks_which_node_it_is(make_sim, channel, config, tmp_path, monkeypatch):
    sim = make_sim(node_ids=(1, 2), freshly_flashed=True)
    ctx = SimContext(config, True, tmp_path, channel)
    asked: list[str] = []

    def answer_for_the_blinking_one(prompt: str) -> str:
        asked.append(prompt)
        assert eventually(lambda: any(node.identifying for node in sim.all_nodes))
        blinking = next(node for node in sim.all_nodes if node.identifying)
        return str(blinking.serial_number - simulated_serial(0))

    monkeypatch.setattr("builtins.input", answer_for_the_blinking_one)

    assert run_address(ctx) == 0
    assert len(asked) == 2
    assert {n: node.serial_number for n, node in sim.nodes.items()} == {1: simulated_serial(1), 2: simulated_serial(2)}
    assert not any(node.identifying for node in sim.all_nodes)


def test_list_runs_end_to_end_through_the_cli(tmp_path: Path, capsys):
    argv = ["--config", str(REPO / "darm.toml"), "--sim", "--state-dir", str(tmp_path), "address", "--list"]
    assert cli.main(argv) == 0
    out = capsys.readouterr().out
    assert all(serial_text(n) in out for n in ROBOT.node_ids)


def test_the_wildcard_serial_is_never_sent(make_sim, channel: str):
    make_sim()
    with CanBus(channel, "virtual") as bus, pytest.raises(ValueError, match="wildcard"):
        assign(bus, 0, 3)


def test_free_node_id_comes_from_the_top():
    assert free_node_id([MAX_NODE_ID, MAX_NODE_ID - 1]) == MAX_NODE_ID - 2
    with pytest.raises(RuntimeError):
        free_node_id(range(MAX_NODE_ID + 1))
